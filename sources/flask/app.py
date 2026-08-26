from flask import Flask, abort, jsonify, redirect, render_template, request, send_file
from werkzeug.middleware.proxy_fix import ProxyFix
from flask_socketio import SocketIO
import json
import os
import signal
import subprocess
import threading
import time
import math
import re
from app_icons import find_app_icon

app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True # Force Flask to bypass template caching
app.wsgi_app = ProxyFix(app.wsgi_app, x_for=1, x_proto=1, x_host=1, x_prefix=1)

# Initialize SocketIO for real-time background telemetry
socketio = SocketIO(app, async_mode='threading')

# Radio front-end CLI (quadrf-fpga) and the Wi-Fi provisioning helper
# (quadrf-network), both overridable for development runs.
SDR_CLI = os.environ.get("QUADRF_JTAG", "/usr/bin/quadrf-jtag")
WIFI_CLI = os.environ.get("QUADRF_APPLY_WIFI", "/usr/sbin/quadrf-apply-wifi")
AP_CLI = os.environ.get("QUADRF_APPLY_AP", "/usr/sbin/quadrf-apply-ap")
HOSTNAME_CLI = os.environ.get("QUADRF_APPLY_HOSTNAME", "/usr/sbin/quadrf-apply-hostname")
HOTSPOT_CLI = os.environ.get("QUADRF_HOTSPOT", "/usr/sbin/quadrf-hotspot")
APP_CLI = os.environ.get("QUADRF_APP", "/usr/sbin/quadrf-app")
UNLOCK_CLI = os.environ.get("QUADRF_TX_UNLOCK", "/usr/sbin/quadrf-tx-unlock")
CONF_PATH = os.environ.get("QUADRF_CONF", "/etc/quadrf/quadrf.conf")
WPA_CONF = os.environ.get("QUADRF_WPA_CONF", "/etc/wpa_supplicant/wpa_supplicant.conf")
TX_FULL_POWER_PATH = os.environ.get("QUADRF_TX_FULL_POWER", "/var/lib/quadrf/tx_full_power")
TX_GAIN_LOCKED_MAX = 40


def tx_full_power_unlocked():
    return os.path.exists(TX_FULL_POWER_PATH)


def load_quadrf_conf(path=CONF_PATH):
    conf = {}
    try:
        with open(path, encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
                    value = value[1:-1]
                conf[key.strip()] = value
    except FileNotFoundError:
        pass
    return conf


ETH_DIRECT = "10.55.1.1"
USB_DIRECT = "10.55.0.1"
AP_DEFAULT = "192.168.44.1"


def _carrier(iface):
    try:
        with open(f"/sys/class/net/{iface}/carrier", encoding="ascii") as fh:
            return fh.read().strip() == "1"
    except OSError:
        return False


def _iface_v4():
    addrs = {}
    try:
        proc = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True, text=True, timeout=1,
        )
    except (OSError, subprocess.TimeoutExpired):
        return addrs
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[2] != "inet":
            continue
        iface = parts[1]
        addr = parts[3].split("/", 1)[0]
        addrs.setdefault(iface, []).append(addr)
    return addrs


def _normalize_ip(addr):
    if not addr:
        return ""
    addr = addr.strip().strip("[]")
    if addr.lower().startswith("::ffff:"):
        addr = addr[7:]
    return addr.split("%", 1)[0]


def _via_header():
    return _normalize_ip(request.headers.get("X-QuadRF-Via", ""))


def get_network_status(via_addr=None):
    """Path-up from addresses/carrier, plus which path accepted this request."""
    conf = load_quadrf_conf()
    ap_addr = conf.get("QUADRF_AP_ADDRESS", AP_DEFAULT) or AP_DEFAULT
    addrs = _iface_v4()
    eth = addrs.get("eth0", [])
    wlan = addrs.get("wlan0", [])
    usb = addrs.get("usb0", [])
    eth_lan_addrs = [a for a in eth if a != ETH_DIRECT]
    wlan_sta_addrs = [a for a in wlan if a != ap_addr]

    eth_carrier = _carrier("eth0")
    paths = {
        "eth_lan": {
            "up": eth_carrier and bool(eth_lan_addrs),
            "addr": eth_lan_addrs[0] if eth_lan_addrs else None,
        },
        "eth_direct": {
            "up": eth_carrier and ETH_DIRECT in eth,
            "addr": ETH_DIRECT if ETH_DIRECT in eth else None,
        },
        "wifi_sta": {
            "up": bool(wlan_sta_addrs),
            "addr": wlan_sta_addrs[0] if wlan_sta_addrs else None,
        },
        "wifi_ap": {
            "up": ap_addr in wlan,
            "addr": ap_addr if ap_addr in wlan else None,
        },
        "usb": {
            "up": USB_DIRECT in usb,
            "addr": USB_DIRECT if USB_DIRECT in usb else None,
        },
    }

    via = None
    via_addr = _normalize_ip(via_addr)
    if via_addr:
        if via_addr == ETH_DIRECT:
            via = "eth_direct"
        elif via_addr == USB_DIRECT:
            via = "usb"
        elif via_addr == ap_addr:
            via = "wifi_ap"
        elif via_addr in eth_lan_addrs:
            via = "eth_lan"
        elif via_addr in wlan_sta_addrs:
            via = "wifi_sta"

    mode = wifi_mode_from_conf(conf)
    fallback = wifi_fallback_from_conf(conf)
    hostname = (conf.get("QUADRF_HOSTNAME") or "quadrf").strip() or "quadrf"
    return {
        "paths": paths,
        "via": via,
        "wifi_mode": mode,
        "wifi_fallback": fallback,
        "wifi_sta_ssid": wifi_sta_ssid(),
        "wifi_fallback_active": mode == "sta" and paths["wifi_ap"]["up"],
        "hostname": hostname,
        "hostname_lock": hostname_lock_from_conf(conf),
    }


HOSTNAME_RE = re.compile(r"^[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?$")


def hostname_lock_from_conf(conf=None):
    conf = conf if conf is not None else load_quadrf_conf()
    value = (conf.get("QUADRF_HOSTNAME_LOCK") or "no").strip().lower()
    return value in ("yes", "true", "1")


def valid_hostname(name):
    return bool(name) and HOSTNAME_RE.fullmatch(name) is not None


def wifi_mode_from_conf(conf=None):
    conf = conf if conf is not None else load_quadrf_conf()
    mode = (conf.get("QUADRF_WIFI_MODE") or "sta").strip().lower()
    return mode if mode in ("sta", "ap", "off") else "sta"


def wifi_fallback_from_conf(conf=None):
    conf = conf if conf is not None else load_quadrf_conf()
    value = (conf.get("QUADRF_WIFI_FALLBACK") or "yes").strip().lower()
    return value not in ("no", "false", "0")


def wifi_sta_ssid(path=None):
    try:
        with open(path or WPA_CONF, encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        return None
    quoted = re.search(r'^\s*ssid="([^"]*)"', text, re.M)
    raw = quoted.group(1) if quoted else None
    if raw is None:
        bare = re.search(r"^\s*ssid=(\S+)", text, re.M)
        raw = bare.group(1) if bare else None
    if not raw or raw.lower() == "none":
        return None
    return raw


def has_saved_station():
    return bool(wifi_sta_ssid())


def run_sudo(cmd, timeout=30, kill_on_timeout=True):
    proc = subprocess.Popen(
        ["sudo", "-n", *cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        if not kill_on_timeout:
            # Hostname apply reloads nginx first, then keeps going. Killing it
            # leaves mDNS on the new name and nginx on the old one (404).
            return subprocess.CompletedProcess(proc.args, 0, "", "still running")
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
        raise
    return subprocess.CompletedProcess(proc.args, proc.returncode, stdout, stderr)


def app_cli(*args):
    return run_sudo([APP_CLI, *args])


def parse_cli_json(proc):
    try:
        return json.loads(proc.stdout or "")
    except json.JSONDecodeError:
        return None

def get_sdr_status():
    """Runs the CLI status commands and parses the output fault-tolerantly."""
    state = {}
    try:
        rx_proc = subprocess.run([SDR_CLI, "--status", "rx"], capture_output=True, text=True)
        if rx_proc.returncode != 0:
            print(f"[Warning] jtag rx status error: {rx_proc.stderr}")
            
        # Keep track of these independently so their print order doesn't matter
        rx_interleave_on = False
        rx_tone_on = False

        for line in rx_proc.stdout.split('\n'):
            line = line.strip()
            if line.startswith("- PLL Lock:"): 
                state['rx_pll_locked'] = "LOCKED" in line
            elif line.startswith("- LO Frequency:"): 
                try: state['rx_freq'] = float(line.split(':', 1)[1].replace('MHz','').strip())
                except ValueError: pass
            elif line.startswith("- Gain:"): 
                try: state['rx_gain'] = float(line.split(':', 1)[1].replace('dB','').strip())
                except ValueError: pass
            elif line.startswith("- Digital Bandwidth (k):"): 
                parts = line.split(':', 1)
                if len(parts) > 1 and "Disabled" not in parts[1]:
                    try: state['rx_bw_k'] = int(parts[1].split('(')[0].strip())
                    except ValueError: pass
            elif line.startswith("- AGC:"): 
                state['rx_agc'] = "Enabled" in line
                if state['rx_agc'] and "Setpoint:" in line:
                    match = re.search(r'Setpoint:\s*(\d+)', line)
                    if match:
                        thr = float(match.group(1))
                        state['rx_gain_dbfs'] = round(20.0 * math.log10(thr / 180.0), 1) if thr > 0 else -40.0
            elif line.startswith("- Interleaved Mode:"): 
                rx_interleave_on = "ON" in line
            elif line.startswith("- Auto Steer:"):
                state['rx_auto_steer'] = "ON" in line
            elif line.startswith("- Polarization:"):
                state['rx_pol'] = "rhcp" if "RHCP" in line.upper() else "lhcp"
            elif line.startswith("- Test Tone:"):
                rx_tone_on = "ON" in line
                state['rx_tone_en'] = rx_tone_on
            elif line.startswith("- Tone Freq:"):
                try: state['rx_tone_freq'] = float(line.split(':', 1)[1].replace('MHz','').strip())
                except ValueError: pass
            elif line.startswith("- Phases:"):
                try:
                    parts = line.split(':', 1)[1].split(',')
                    state['rx_p1'] = float(parts[0].strip())
                    state['rx_p2'] = float(parts[1].strip())
                    state['rx_p3'] = float(parts[2].strip())
                    state['rx_p4'] = float(parts[3].strip())
                except Exception: pass

        # Resolve the Rx dropdown state definitively at the end
        if rx_tone_on:
            state['rx_antenna_mode'] = "test"
        else:
            state['rx_antenna_mode'] = "4" if rx_interleave_on else "1"

        tx_proc = subprocess.run([SDR_CLI, "--status", "tx"], capture_output=True, text=True)
        if tx_proc.returncode != 0:
            print(f"[Warning] jtag tx status error: {tx_proc.stderr}")
            
        for line in tx_proc.stdout.split('\n'):
            line = line.strip()
            if line.startswith("- PLL Lock:"): 
                state['tx_pll_locked'] = "LOCKED" in line
            elif line.startswith("- Tx is"): 
                state['tx_on'] = "ON" in line
            elif line.startswith("- LO Frequency:"): 
                try: state['tx_freq'] = float(line.split(':', 1)[1].replace('MHz','').strip())
                except ValueError: pass
            elif line.startswith("- Gain:"): 
                try: state['tx_gain'] = float(line.split(':', 1)[1].replace('dB','').strip())
                except ValueError: pass
            elif line.startswith("- Antennas enabled:"):
                parts = line.split(':', 1)
                if len(parts) > 1:
                    ants = parts[1].strip()
                    state['tx_ant1'] = '1' in ants
                    state['tx_ant2'] = '2' in ants
                    state['tx_ant3'] = '3' in ants
                    state['tx_ant4'] = '4' in ants
            elif line.startswith("- Tx follow Rx:"):
                state['tx_follow_rx'] = "ON" in line
            elif line.startswith("- Test Tone:"):
                state['tx_tone_en'] = "ON" in line
            elif line.startswith("- Tone Freq:"):
                try: state['tx_tone_freq'] = float(line.split(':', 1)[1].replace('MHz','').strip())
                except ValueError: pass
            elif line.startswith("- Phases:"):
                try:
                    parts = line.split(':', 1)[1].split(',')
                    state['tx_p1'] = float(parts[0].strip())
                    state['tx_p2'] = float(parts[1].strip())
                    state['tx_p3'] = float(parts[2].strip())
                    state['tx_p4'] = float(parts[3].strip())
                except Exception: pass

        state['tx_full_power'] = tx_full_power_unlocked()
                    
    except Exception as e:
        print(f"Status read error: {e}")
        state['tx_full_power'] = tx_full_power_unlocked()
        
    return state

@app.route('/', methods=['GET'])
def index():
    conf = load_quadrf_conf()
    hostname = conf.get('QUADRF_HOSTNAME', 'quadrf')
    net = get_network_status(_via_header())
    return render_template(
        'index.html',
        ap_ssid=conf.get('QUADRF_AP_SSID', 'QuadRF'),
        desktop_host=f"{hostname}d.local",
        hostname=hostname,
        hostname_lock=net["hostname_lock"],
        net_paths=net["paths"],
        net_via=net["via"],
        wifi_mode=net["wifi_mode"],
        wifi_fallback=net["wifi_fallback"],
        wifi_sta_ssid=net["wifi_sta_ssid"] or '',
        wifi_fallback_active=net["wifi_fallback_active"],
    )

@app.route('/split')
def split():
    hostname = load_quadrf_conf().get('QUADRF_HOSTNAME', 'quadrf')
    raw_host = request.host or ''
    host_only = raw_host.split(':')[0]
    port = raw_host.split(':')[1] if ':' in raw_host else ''
    desktop_names = {
        f"{hostname}d.local",
        f"{hostname}-desktop.local",
        f"desktop.{hostname}.local",
    }
    if host_only.lower() in {n.lower() for n in desktop_names} or port == '6080':
        return render_template('split.html')
    scheme = request.headers.get('X-Forwarded-Proto') or request.scheme or 'http'
    return redirect(f"{scheme}://{hostname}d.local/split", code=302)

@app.route('/api/status', methods=['GET'])
def get_status():
    state = get_sdr_status()
    state.update(get_network_status(_via_header()))
    return jsonify(state)


def _json_body():
    return request.get_json(silent=True) or {}


def _cli_error(proc, fallback):
    return (proc.stderr or proc.stdout or fallback).strip()


@app.route('/api/network/mode', methods=['POST'])
def network_mode():
    mode = (_json_body().get("mode") or "").strip().lower()
    if mode not in ("sta", "ap", "off"):
        return jsonify({"status": "error", "message": "mode must be sta, ap, or off"}), 400
    if mode == "sta" and not has_saved_station():
        return jsonify({
            "status": "error",
            "need_config": True,
            "message": "Save a network first.",
        }), 400
    try:
        proc = run_sudo([HOTSPOT_CLI, mode], timeout=40)
    except subprocess.TimeoutExpired:
        return jsonify({"status": "error", "message": "Wi-Fi switch timed out"}), 500
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": _cli_error(proc, "Wi-Fi switch failed")}), 500
    return jsonify({"status": "ok", **get_network_status(_via_header())})


@app.route('/api/network/fallback', methods=['POST'])
def network_fallback():
    enabled = _json_body().get("enabled")
    if not isinstance(enabled, bool):
        return jsonify({"status": "error", "message": "enabled must be true or false"}), 400
    proc = run_sudo([HOTSPOT_CLI, "fallback", "yes" if enabled else "no"])
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": _cli_error(proc, "failed to set fallback")}), 500
    return jsonify({"status": "ok", **get_network_status(_via_header())})


@app.route('/api/network/station', methods=['POST'])
def network_station():
    body = _json_body()
    ssid = (body.get("ssid") or "").strip()
    password = body.get("password") or ""
    if not ssid:
        return jsonify({"status": "error", "message": "SSID is required"}), 400
    proc = run_sudo([WIFI_CLI, ssid, password] if password else [WIFI_CLI, ssid])
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": _cli_error(proc, "failed to save client network")}), 500
    return jsonify({"status": "ok", **get_network_status(_via_header())})


@app.route('/api/network/hotspot', methods=['POST'])
def network_hotspot():
    body = _json_body()
    ssid = (body.get("ssid") or "").strip()
    password = body.get("password") or ""
    if not ssid:
        return jsonify({"status": "error", "message": "Hotspot SSID is required"}), 400
    proc = run_sudo([AP_CLI, ssid, password] if password else [AP_CLI, ssid])
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": _cli_error(proc, "failed to save hotspot")}), 500
    return jsonify({"status": "ok", **get_network_status(_via_header())})


@app.route('/api/network/hostname', methods=['POST'])
def network_hostname():
    body = _json_body()
    if "lock" in body:
        if not isinstance(body["lock"], bool):
            return jsonify({"status": "error", "message": "lock must be true or false"}), 400
        proc = run_sudo([HOSTNAME_CLI, "lock", "yes" if body["lock"] else "no"])
        if proc.returncode != 0:
            return jsonify({"status": "error", "message": _cli_error(proc, "failed to set name lock")}), 500
        return jsonify({"status": "ok", **get_network_status(_via_header())})
    name = (body.get("name") or "").strip().lower()
    if name:
        if not valid_hostname(name):
            return jsonify({
                "status": "error",
                "message": "name must be a hostname label (letters, digits, hyphen)",
            }), 400
        try:
            proc = run_sudo([HOSTNAME_CLI, "set", name], timeout=90, kill_on_timeout=False)
        except subprocess.TimeoutExpired:
            return jsonify({"status": "ok", **get_network_status(_via_header())})
        if proc.returncode != 0:
            return jsonify({"status": "error", "message": _cli_error(proc, "failed to set hostname")}), 500
        return jsonify({"status": "ok", **get_network_status(_via_header())})
    if body.get("reset"):
        try:
            proc = run_sudo([HOSTNAME_CLI, "reset"], timeout=90, kill_on_timeout=False)
        except subprocess.TimeoutExpired:
            return jsonify({"status": "ok", **get_network_status(_via_header())})
        if proc.returncode != 0:
            return jsonify({"status": "error", "message": _cli_error(proc, "failed to reset hostname")}), 500
        return jsonify({"status": "ok", **get_network_status(_via_header())})
    return jsonify({"status": "error", "message": "lock, name, or reset required"}), 400

def broadcast_full_status():
    """Fetches the latest state and pushes it to all connected WebSocket clients."""
    state = get_sdr_status()
    socketio.emit('sdr_status', state)

def broadcast_app_state():
    proc = app_cli("status")
    payload = parse_cli_json(proc)
    if payload is None:
        payload = {"status": "error", "apps": [], "message": (proc.stderr or proc.stdout or "").strip()}
    socketio.emit("app_state", payload)
    return payload, proc

def requested_app_id():
    if request.is_json and request.json:
        value = request.json.get("app")
        if value:
            return str(value)
    return request.args.get("app", "").strip()


def requested_client_id():
    if request.is_json and request.json:
        value = request.json.get("client")
        if value:
            return str(value)[:64]
    return (request.args.get("client") or "").strip()[:64]


# Spatial RF Vision tabs register here. Last client gone -> stop after a
# short grace so a refresh can re-attach. A watchdog also prunes stale
# entries; pagehide beacons are best-effort and often never arrive.
AR_CLIENTS = {}
AR_SEEN = False
AR_STOP_TIMER = None
AR_LOCK = threading.Lock()
AR_STOP_GRACE = 0.8
AR_STALE_SEC = 2.5


def _prune_ar_clients(now=None):
    now = now if now is not None else time.time()
    for key, seen in list(AR_CLIENTS.items()):
        if now - seen > AR_STALE_SEC:
            AR_CLIENTS.pop(key, None)


def _cancel_ar_stop():
    global AR_STOP_TIMER
    if AR_STOP_TIMER is not None:
        AR_STOP_TIMER.cancel()
        AR_STOP_TIMER = None


def _stop_ar_if_idle():
    global AR_SEEN
    with AR_LOCK:
        _prune_ar_clients()
        if AR_CLIENTS:
            return
        AR_SEEN = False
    proc = app_cli("stop", "ar")
    payload = parse_cli_json(proc)
    if payload:
        socketio.emit("app_state", payload)
    else:
        broadcast_app_state()


def _schedule_ar_stop(reset=True):
    global AR_STOP_TIMER
    if reset:
        _cancel_ar_stop()
    elif AR_STOP_TIMER is not None:
        return

    AR_STOP_TIMER = threading.Timer(AR_STOP_GRACE, _stop_ar_if_idle)
    AR_STOP_TIMER.daemon = True
    AR_STOP_TIMER.start()


def _ar_watchdog_loop():
    while True:
        time.sleep(1.0)
        with AR_LOCK:
            _prune_ar_clients()
            idle = AR_SEEN and not AR_CLIENTS
        if idle:
            _schedule_ar_stop(reset=False)


threading.Thread(target=_ar_watchdog_loop, name="ar-watchdog", daemon=True).start()


@app.route('/api/apps', methods=['GET'])
def apps_status():
    proc = app_cli("status")
    payload = parse_cli_json(proc)
    if payload is None:
        return jsonify({"status": "error", "message": (proc.stderr or proc.stdout or "app status failed").strip()}), 500
    return jsonify(payload)


@app.route('/api/apps/icon/<icon>', methods=['GET'])
def app_icon(icon):
    path = find_app_icon(icon)
    if path is None:
        abort(404)
    response = send_file(path, conditional=True)
    response.cache_control.public = True
    response.cache_control.max_age = 86400
    return response


@app.route('/api/apps/start', methods=['POST'])
def apps_start():
    app_id = requested_app_id()
    if not app_id:
        return jsonify({"status": "error", "message": "app is required"}), 400
    proc = app_cli("start", app_id)
    payload = parse_cli_json(proc)
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": (proc.stderr or proc.stdout or "start failed").strip()}), 500
    if payload:
        socketio.emit("app_state", payload)
        return jsonify(payload)
    payload, _ = broadcast_app_state()
    return jsonify(payload)

@app.route('/api/apps/attach', methods=['POST'])
def apps_attach():
    global AR_SEEN
    app_id = requested_app_id() or "ar"
    if app_id != "ar":
        return jsonify({"status": "error", "message": "attach is only used by ar"}), 400
    client = requested_client_id() or "anon"
    with AR_LOCK:
        AR_SEEN = True
        AR_CLIENTS[client] = time.time()
        _cancel_ar_stop()
    return jsonify({"status": "ok"})


@app.route('/api/apps/detach', methods=['POST'])
def apps_detach():
    app_id = requested_app_id() or "ar"
    if app_id != "ar":
        return jsonify({"status": "error", "message": "detach is only used by ar"}), 400
    client = requested_client_id()
    with AR_LOCK:
        if client:
            AR_CLIENTS.pop(client, None)
        _prune_ar_clients()
        if not AR_CLIENTS:
            _schedule_ar_stop()
    return jsonify({"status": "ok"})


@app.route('/api/apps/stop', methods=['POST'])
def apps_stop():
    global AR_SEEN
    app_id = requested_app_id()
    if not app_id:
        return jsonify({"status": "error", "message": "app is required"}), 400
    if app_id == "ar":
        with AR_LOCK:
            AR_CLIENTS.clear()
            AR_SEEN = False
            _cancel_ar_stop()
    proc = app_cli("stop", app_id)
    payload = parse_cli_json(proc)
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": (proc.stderr or proc.stdout or "stop failed").strip()}), 500
    if payload:
        socketio.emit("app_state", payload)
        return jsonify(payload)
    payload, _ = broadcast_app_state()
    return jsonify(payload)

@app.route('/api/control', methods=['POST'])
def control_sdr():
    data = request.json
    control_type = data.get('type')
    value = data.get('value')
    
    cmd = [SDR_CLI]

    try:
        # --- RECEIVE (Rx) COMMANDS ---
        if control_type == 'rx_reset':
            cmd.extend(["--rx", "off"])
        elif control_type == 'rx_freq':
            cmd.extend(["--rx", f"freq={float(value)}"])            
        elif control_type == 'rx_gain':
            cmd.extend(["--rx", f"gain={int(float(value))}"])
        elif control_type == 'rx_agc':
            cmd.extend(["--rx", f"agc={float(value)}"])
        elif control_type == 'rx_pol':
            cmd.extend(["--rx", f"pol={value}"])
        elif control_type == 'rx_bw':
            cmd.extend(["--rx", f"bw={float(value)}"])
        elif control_type == 'rx_antenna_mode':
            if str(value) == "4":
                cmd.extend(["--rx", "antennas=15,interleave=1,tone_en=0"])
            elif str(value) == "1":
                cmd.extend(["--rx", "antennas=1,interleave=0,tone_en=0"])
            elif str(value) == "test":
                cmd.extend(["--rx", "tone_en=1"])
        elif control_type == 'rx_auto_steer':
            cmd.extend(["--rx", f"autosteer={1 if value else 0}"])
        elif control_type == 'rx_tone':
            cmd.extend(["--rx", f"tone_freq={float(value)}"])
        elif control_type == 'rx_phases':
            cmd.extend(["--rx", f"p1={float(value['p1']):.1f},p2={float(value['p2']):.1f},p3={float(value['p3']):.1f},p4={float(value['p4']):.1f}"])

        # --- TRANSMIT (Tx) COMMANDS ---
        elif control_type == 'tx_reset':
            cmd.extend(["--tx", "off"])
        elif control_type == 'tx_follow_rx':
            cmd.extend(["--tx", f"tx_follow_rx={1 if value else 0}"])
        elif control_type == 'tx_on_off':
            if value:
                cmd.extend(["--tx", "antennas=15"]) 
            else:
                cmd.extend(["--tx", "off"])
        elif control_type == 'tx_freq':
            cmd.extend(["--tx", f"freq={float(value)}"])            
        elif control_type == 'tx_gain':
            gain = int(float(value))
            if not tx_full_power_unlocked() and gain > TX_GAIN_LOCKED_MAX:
                return jsonify({
                    "status": "tx_limited",
                    "message": "TX gain limited until the lawful-use notice is accepted.",
                    "tx_full_power": False,
                    "max": TX_GAIN_LOCKED_MAX,
                }), 403
            cmd.extend(["--tx", f"gain={gain}"])
        elif control_type == 'tx_ant_enables':
            mask = 0
            if value.get('a1'): mask |= 1
            if value.get('a2'): mask |= 2
            if value.get('a3'): mask |= 4
            if value.get('a4'): mask |= 8
            cmd.extend(["--tx", f"antennas={mask}"])
        elif control_type == 'tx_mode':
            if str(value) == "test":
                cmd.extend(["--tx", "tone_en=1"])
            else:
                cmd.extend(["--tx", "tone_en=0"])
        elif control_type == 'tx_tone':
            cmd.extend(["--tx", f"tone_freq={float(value)}"])
        elif control_type == 'tx_phases':
            cmd.extend(["--tx", f"p1={float(value['p1']):.1f},p2={float(value['p2']):.1f},p3={float(value['p3']):.1f},p4={float(value['p4']):.1f}"])

        # --- UNSUPPORTED / MOCKED FEATURES ---
        if control_type in ['rx_analog_bw']:
            return jsonify({"status": "success", "executed": f"{control_type} (Mocked)", "output": ""})
            
        print("Executing:", " ".join(cmd))
        result = subprocess.run(cmd, check=True, text=True, capture_output=True)
        
        def delayed_broadcast():
            time.sleep(0.1)
            broadcast_full_status()
        threading.Thread(target=delayed_broadcast).start()
        
        return jsonify({"status": "success", "executed": " ".join(cmd), "output": result.stdout})

    except subprocess.CalledProcessError as e:
        return jsonify({"status": "error", "message": e.stderr}), 500
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/api/tx_unlock', methods=['POST'])
def tx_unlock():
    proc = run_sudo([UNLOCK_CLI])
    if proc.returncode != 0:
        return jsonify({"status": "error", "message": _cli_error(proc, "failed to unlock TX")}), 500
    return jsonify({"status": "ok", "tx_full_power": True})


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True)
