from flask import Flask, render_template, request, jsonify, redirect
from werkzeug.middleware.proxy_fix import ProxyFix
from flask_socketio import SocketIO
import json
import os
import subprocess
import threading
import time
import math
import re

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
APP_CLI = os.environ.get("QUADRF_APP", "/usr/sbin/quadrf-app")
CONF_PATH = os.environ.get("QUADRF_CONF", "/etc/quadrf/quadrf.conf")


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


def hotspot_is_up():
    return subprocess.run(["systemctl", "is-active", "--quiet", "hostapd.service"]).returncode == 0


def run_sudo(cmd):
    return subprocess.run(["sudo", "-n", *cmd], capture_output=True, text=True)


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
                    
    except Exception as e:
        print(f"Status read error: {e}")
        
    return state

@app.route('/', methods=['GET', 'POST'])
def index():
    wifi_success_msg = None
    wifi_error_msg = None
    conf = load_quadrf_conf()

    if request.method == 'POST':
        intent = request.form.get('intent', 'wifi')
        if intent == 'ap':
            ssid = request.form.get('ap_ssid', '').strip()
            password = request.form.get('ap_password', '')
            if not ssid:
                wifi_error_msg = "Hotspot SSID is required."
            else:
                try:
                    proc = run_sudo([AP_CLI, ssid, password] if password else [AP_CLI, ssid])
                    if proc.returncode != 0:
                        wifi_error_msg = (proc.stderr or proc.stdout or "Failed to apply hotspot settings.").strip()
                    else:
                        conf = load_quadrf_conf()
                        note = (proc.stdout or "").strip()
                        wifi_success_msg = note or f"Hotspot SSID set to '{ssid}'."
                        if hotspot_is_up():
                            wifi_success_msg += " Devices on this access point need to reconnect."
                except Exception as e:
                    wifi_error_msg = f"Failed to apply hotspot settings: {e}"
        else:
            ssid = request.form.get('ssid', '').strip()
            password = request.form.get('password', '').strip()

            if not ssid:
                wifi_error_msg = "SSID is required to connect to a new network."
            else:
                try:
                    log_file = open('/tmp/wifi_debug.log', 'w')
                    subprocess.Popen(
                        ['sudo', WIFI_CLI, ssid, password],
                        stdout=log_file,
                        stderr=subprocess.STDOUT
                    )
                    wifi_success_msg = f"Network settings applied! QuadRF is rebooting to connect to '{ssid}'. Please switch your device to use this same new network and refresh this page in ~30 seconds."
                except Exception as e:
                    wifi_error_msg = f"Failed to execute network script: {e}"
    hostname = conf.get('QUADRF_HOSTNAME', 'quadrf')
    return render_template(
        'index.html',
        wifi_success_msg=wifi_success_msg,
        wifi_error_msg=wifi_error_msg,
        ap_ssid=conf.get('QUADRF_AP_SSID', 'QuadRF'),
        hotspot_active=hotspot_is_up(),
        desktop_host=f"{hostname}d.local",
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
    return jsonify(get_sdr_status())

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
            cmd.extend(["--tx", f"gain={int(float(value))}"])
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

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True)
