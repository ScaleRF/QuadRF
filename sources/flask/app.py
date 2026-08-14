from flask import Flask, render_template, request, jsonify
from werkzeug.middleware.proxy_fix import ProxyFix
from flask_socketio import SocketIO
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

    if request.method == 'POST':
        ssid = request.form.get('ssid', '').strip()
        password = request.form.get('password', '').strip()

        if not ssid:
            wifi_error_msg = "SSID is required to connect to a new network."
        else:
            try:
                # Open a log file to catch background errors
                log_file = open('/tmp/wifi_debug.log', 'w')
                
                # Execute the bash script and pipe stdout/stderr to the log
                subprocess.Popen(
                    ['sudo', WIFI_CLI, ssid, password],
                    stdout=log_file,
                    stderr=subprocess.STDOUT
                )
                
                wifi_success_msg = f"Network settings applied! QuadRF is rebooting to connect to '{ssid}'. Please switch your device to use this same new network and refresh this page in ~30 seconds."
            except Exception as e:
                wifi_error_msg = f"Failed to execute network script: {e}"
    return render_template('index.html', wifi_success_msg=wifi_success_msg, wifi_error_msg=wifi_error_msg)

@app.route('/split')
def split():
    return render_template('split.html')

@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify(get_sdr_status())

def broadcast_full_status():
    """Fetches the latest state and pushes it to all connected WebSocket clients."""
    state = get_sdr_status()
    socketio.emit('sdr_status', state)

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
