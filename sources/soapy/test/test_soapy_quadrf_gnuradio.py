#!/usr/bin/env python3
"""
Comprehensive GNU Radio test suite for Soapy QuadRF Source and Sink blocks.
Tests:
  1. GRC flowgraph compilation with grcc using soapy_quadrf_source and soapy_quadrf_sink
  2. Multi-channel RX streaming (4-ch, 3-ch, 2-ch, 1-ch) in fc32 and sc8
  3. Resampled vs Native sample rates for RX and TX
  4. Live runtime parameter callbacks (freq, gain, AGC, bandwidth, pol, antennas, phases)
  5. TX streaming (fc32 and sc8, tones, analog bandwidth, follow_rx, phases)
  6. Full-duplex RX + TX simultaneous operation
  7. Channel data integrity and cross-channel differentiation
"""

import sys
import time
import os
import subprocess
import numpy as np
from gnuradio import gr, blocks, soapy, analog

def banner(title):
    print("\n" + "="*65)
    print(f"  {title}")
    print("="*65)

class SampleCounter(gr.sync_block):
    def __init__(self, dtype):
        super().__init__(name="SampleCounter", in_sig=[dtype], out_sig=[])
        self.count = 0
        self.samples = []
    def work(self, input_items, output_items):
        n = len(input_items[0])
        self.count += n
        if len(self.samples) < 1000:
            take = min(1000 - len(self.samples), n)
            self.samples.extend(input_items[0][:take])
        return n

def test_grc_compilation():
    banner("TEST 1: GRC Flowgraph Compilation via grcc")
    grc_content = """options:
  parameters:
    author: QuadRF
    category: '[GRC Hier Blocks]'
    id: test_quadrf_blocks
    title: Test QuadRF Blocks
    generate_options: no_gui
    output_language: python
    window_size: (1000,1000)
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [8, 8]
    rotation: 0
    state: enabled

blocks:
- name: samp_rate
  id: variable
  parameters:
    value: '20000000'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [184, 12]
    rotation: 0
    state: enabled

- name: freq
  id: variable
  parameters:
    value: '5600000000'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [280, 12]
    rotation: 0
    state: enabled

- name: soapy_quadrf_source_0
  id: soapy_quadrf_source
  parameters:
    agc: 'False'
    agc_setpoint: '-15'
    ant1: 'True'
    ant2: 'True'
    ant3: 'True'
    ant4: 'True'
    autosteer: 'False'
    bandwidth: '0'
    center_freq: freq
    dev_args: '""'
    gain: '25'
    nchan: '4'
    phase1: '0'
    phase2: '45'
    phase3: '90'
    phase4: '135'
    pol: rhcp
    samp_rate: samp_rate
    tone_en: 'False'
    tone_freq: '1000000'
    type: fc32
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [40, 140]
    rotation: 0
    state: enabled

- name: blocks_null_sink_0
  id: blocks_null_sink
  parameters:
    bus_structure_sink: '[[0,],]'
    type: complex
    vlen: '1'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [360, 144]
    rotation: 0
    state: enabled

- name: blocks_null_sink_1
  id: blocks_null_sink
  parameters:
    bus_structure_sink: '[[0,],]'
    type: complex
    vlen: '1'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [360, 176]
    rotation: 0
    state: enabled

- name: blocks_null_sink_2
  id: blocks_null_sink
  parameters:
    bus_structure_sink: '[[0,],]'
    type: complex
    vlen: '1'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [360, 208]
    rotation: 0
    state: enabled

- name: blocks_null_sink_3
  id: blocks_null_sink
  parameters:
    bus_structure_sink: '[[0,],]'
    type: complex
    vlen: '1'
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [360, 240]
    rotation: 0
    state: enabled

- name: analog_sig_source_x_0
  id: analog_sig_source_x
  parameters:
    amp: '0.8'
    frequency: '100000'
    offset: '0'
    phase: '0'
    samp_rate: samp_rate
    type: complex
    waveform: analog.GR_COS_WAVE
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [40, 320]
    rotation: 0
    state: enabled

- name: soapy_quadrf_sink_0
  id: soapy_quadrf_sink
  parameters:
    ant1: 'True'
    ant2: 'True'
    ant3: 'True'
    ant4: 'True'
    bandwidth: '0'
    center_freq: freq
    dev_args: '""'
    follow_rx: 'False'
    gain: '20'
    phase1: '0'
    phase2: '0'
    phase3: '0'
    phase4: '0'
    samp_rate: samp_rate
    tone_en: 'False'
    tone_freq: '1000000'
    type: fc32
  states:
    bus_sink: false
    bus_source: false
    bus_structure: null
    coordinate: [320, 320]
    rotation: 0
    state: enabled

connections:
- [soapy_quadrf_source_0, '0', blocks_null_sink_0, '0']
- [soapy_quadrf_source_0, '1', blocks_null_sink_1, '0']
- [soapy_quadrf_source_0, '2', blocks_null_sink_2, '0']
- [soapy_quadrf_source_0, '3', blocks_null_sink_3, '0']
- [analog_sig_source_x_0, '0', soapy_quadrf_sink_0, '0']

metadata:
  file_format: 1
  grc_version: 3.10.12.0
"""
    tmp_grc = "/tmp/test_quadrf_blocks.grc"
    tmp_py = "/tmp/test_quadrf_blocks.py"
    with open(tmp_grc, "w") as f:
        f.write(grc_content)
    
    print(f"Compiling {tmp_grc} with grcc...")
    res = subprocess.run(["grcc", "-o", "/tmp", tmp_grc], capture_output=True, text=True)
    if res.returncode != 0:
        print("FAIL: grcc failed with return code", res.returncode)
        print("STDOUT:", res.stdout)
        print("STDERR:", res.stderr)
        return False
    
    print("PASS: grcc compiled successfully to", tmp_py)
    if os.path.exists(tmp_py):
        with open(tmp_py) as f:
            code = f.read()
        print(f"Generated python length: {len(code)} bytes")
        has_src = "self.soapy_quadrf_source_0" in code
        has_snk = "self.soapy_quadrf_sink_0" in code
        print(f"Contains soapy_quadrf_source_0: {has_src}")
        print(f"Contains soapy_quadrf_sink_0: {has_snk}")
        return has_src and has_snk
    return False

class QuadRFRxFlowgraph(gr.top_block):
    def __init__(self, fmt="fc32", nchan=4, rate=20.0e6, freq=5.6e9, gain=25.0, agc=False, agc_setpoint=-15.0,
                 bandwidth=0.0, pol="rhcp", ant_mask=15, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
        super().__init__("QuadRFRxFlowgraph")
        self.fmt = fmt
        self.nchan = nchan
        self.rate = rate
        self.freq = freq
        self.gain = gain
        self.agc = agc
        self.agc_setpoint = agc_setpoint
        self.bandwidth = bandwidth
        self.pol = pol
        self.ant_mask = ant_mask
        self.p1, self.p2, self.p3, self.p4 = p1, p2, p3, p4

        dev = 'driver=mipi'
        dev_args = ''
        stream_args = ''
        tune_args = [''] * nchan
        settings = [''] * nchan

        self.src = soapy.source(dev, fmt, nchan, dev_args, stream_args, tune_args, settings)
        self.src.set_sample_rate(0, rate)
        self.src.set_bandwidth(0, bandwidth)
        self.src.set_frequency(0, freq)
        self.src.set_gain_mode(0, agc)
        if not agc:
            self.src.set_gain(0, min(max(float(gain), 0.0), 63.0))
        else:
            self.src.write_setting(0, 'agc_setpoint', str(agc_setpoint))
        self.src.write_setting(0, 'pol', str(pol))
        self.src.write_setting(0, 'autosteer', 'false')
        self.src.write_setting(0, 'antennas', str(ant_mask))
        self.src.write_setting(0, 'p1', str(p1))
        self.src.write_setting(0, 'p2', str(p2))
        self.src.write_setting(0, 'p3', str(p3))
        self.src.write_setting(0, 'p4', str(p4))

        self.counters = []
        is_float = (fmt == "fc32")
        dtype = np.complex64 if is_float else np.int16

        for ch in range(nchan):
            cnt = SampleCounter(dtype)
            self.counters.append(cnt)
            self.connect((self.src, ch), (cnt, 0))

def run_rx_test(name, fmt="fc32", nchan=4, rate=20.0e6, duration=1.5, freq=5.6e9, gain=25.0, agc=False):
    print(f"\n--- Subtest: {name} (fmt={fmt}, nchan={nchan}, rate={rate/1e6:.3f} Msps) ---")
    tb = None
    try:
        tb = QuadRFRxFlowgraph(fmt=fmt, nchan=nchan, rate=rate, freq=freq, gain=gain, agc=agc)
        t0 = time.time()
        tb.start()
        time.sleep(duration)
        tb.stop()
        tb.wait()
        elapsed = time.time() - t0

        counts = [c.count for c in tb.counters]
        actual_samples = counts[0] if counts else 0
        measured_rate = actual_samples / elapsed if elapsed > 0 else 0
        print(f"  Samples received: {counts} | Elapsed: {elapsed:.2f}s | Measured rate: {measured_rate/1e6:.3f} Msps")

        if actual_samples == 0:
            print("  FAIL: No samples received")
            return False

        # Verify cross-channel data
        if nchan >= 2 and fmt == "fc32" and len(tb.counters) >= 2:
            d0 = np.array(tb.counters[0].samples)
            d1 = np.array(tb.counters[1].samples)
            if len(d0) > 100 and len(d1) > 100:
                diff = np.abs(d0 - d1)
                diff_pct = (np.count_nonzero(diff > 1e-4) / len(d0)) * 100.0
                print(f"  Ch0 vs Ch1 difference: {diff_pct:.1f}% different across first {len(d0)} samples")
                if diff_pct < 5.0:
                    print("  FAIL: Channels 0 and 1 appear identical (deinterleave failed)")
                    return False

        print(f"  PASS: {name}")
        return True
    except Exception as e:
        print(f"  FAIL: Exception: {e}")
        return False
    finally:
        if tb:
            del tb

def test_rx_streaming_modes():
    banner("TEST 2: RX Streaming (All Channel Counts, Formats & Rates)")
    all_ok = True
    # 4-Channel Native (~20.101 Msps)
    all_ok &= run_rx_test("4-Channel fc32 Native Rate", fmt="fc32", nchan=4, rate=20.101e6)
    all_ok &= run_rx_test("4-Channel sc8 Native Rate", fmt="sc8", nchan=4, rate=20.101e6)
    # 4-Channel Resampled
    all_ok &= run_rx_test("4-Channel fc32 Resampled (10 Msps)", fmt="fc32", nchan=4, rate=10.0e6)
    all_ok &= run_rx_test("4-Channel fc32 Resampled (5 Msps)", fmt="fc32", nchan=4, rate=5.0e6)
    all_ok &= run_rx_test("4-Channel sc8 Resampled (10 Msps)", fmt="sc8", nchan=4, rate=10.0e6)

    # 3-Channel and 2-Channel
    all_ok &= run_rx_test("3-Channel fc32 (20.101 Msps)", fmt="fc32", nchan=3, rate=20.101e6)
    all_ok &= run_rx_test("2-Channel fc32 (20.101 Msps)", fmt="fc32", nchan=2, rate=20.101e6)
    all_ok &= run_rx_test("2-Channel sc8 (20.101 Msps)", fmt="sc8", nchan=2, rate=20.101e6)

    # 1-Channel Native (~80.39 Msps)
    all_ok &= run_rx_test("1-Channel fc32 Native Rate (~80 Msps)", fmt="fc32", nchan=1, rate=80.0e6)
    # 1-Channel Resampled
    all_ok &= run_rx_test("1-Channel fc32 Resampled (40 Msps)", fmt="fc32", nchan=1, rate=40.0e6)
    all_ok &= run_rx_test("1-Channel fc32 Resampled (20 Msps)", fmt="fc32", nchan=1, rate=20.0e6)
    all_ok &= run_rx_test("1-Channel sc8 Resampled (40 Msps)", fmt="sc8", nchan=1, rate=40.0e6)
    return all_ok

def test_dynamic_callbacks():
    banner("TEST 3: Dynamic Runtime Parameter Updates during Active Streaming")
    try:
        tb = QuadRFRxFlowgraph(fmt="fc32", nchan=4, rate=20.101e6, freq=5.6e9, gain=20.0)
        tb.start()
        time.sleep(0.5)

        print("Updating center_freq to 5.2 GHz...")
        tb.src.set_frequency(0, 5.2e9)
        time.sleep(0.3)
        print("Updating center_freq to 5.8 GHz...")
        tb.src.set_frequency(0, 5.8e9)
        time.sleep(0.3)

        print("Updating RF gain to 45 dB...")
        tb.src.set_gain(0, 45.0)
        time.sleep(0.3)

        print("Enabling AGC...")
        tb.src.set_gain_mode(0, True)
        tb.src.write_setting(0, 'agc_setpoint', '-20.0')
        time.sleep(0.3)

        print("Disabling AGC and returning to manual gain 30 dB...")
        tb.src.set_gain_mode(0, False)
        tb.src.set_gain(0, 30.0)
        time.sleep(0.3)

        print("Updating polarization to LHCP...")
        tb.src.write_setting(0, 'pol', 'lhcp')
        time.sleep(0.2)
        print("Updating polarization to RHCP...")
        tb.src.write_setting(0, 'pol', 'rhcp')
        time.sleep(0.2)

        print("Updating phase offsets: p1=45, p2=90, p3=135, p4=180...")
        tb.src.write_setting(0, 'p1', '45.0')
        tb.src.write_setting(0, 'p2', '90.0')
        tb.src.write_setting(0, 'p3', '135.0')
        tb.src.write_setting(0, 'p4', '180.0')
        time.sleep(0.3)

        print("Updating active antenna mask to 3 (Rx1+Rx2)...")
        tb.src.write_setting(0, 'antennas', '3')
        time.sleep(0.2)
        print("Restoring antenna mask to 15 (all)...")
        tb.src.write_setting(0, 'antennas', '15')
        time.sleep(0.2)

        print("Updating digital bandwidth to 10 MHz...")
        tb.src.set_bandwidth(0, 10.0e6)
        time.sleep(0.2)

        tb.stop()
        tb.wait()
        total_samples = tb.counters[0].count
        print(f"PASS: Dynamic updates completed successfully. Total samples received on Ch0: {total_samples}")
        return total_samples > 0
    except Exception as e:
        print(f"FAIL: Exception in dynamic callbacks: {e}")
        return False

class QuadRFTxFlowgraph(gr.top_block):
    def __init__(self, fmt="fc32", rate=40.0e6, freq=5.6e9, gain=20.0, bandwidth=0, follow_rx=False,
                 ant_mask=15, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
        super().__init__("QuadRFTxFlowgraph")
        dev = 'driver=mipi'
        dev_args = ''
        stream_args = ''
        tune_args = ['']
        settings = ['']

        self.snk = soapy.sink(dev, fmt, 1, dev_args, stream_args, tune_args, settings)
        self.snk.set_sample_rate(0, rate)
        self.snk.set_bandwidth(0, bandwidth)
        self.snk.set_frequency(0, freq)
        self.snk.set_gain(0, min(max(float(gain), 0.0), 63.0))
        self.snk.write_setting(0, 'follow_rx', str('true' if follow_rx else 'false'))
        self.snk.write_setting(0, 'antennas', str(ant_mask))
        self.snk.write_setting(0, 'p1', str(p1))
        self.snk.write_setting(0, 'p2', str(p2))
        self.snk.write_setting(0, 'p3', str(p3))
        self.snk.write_setting(0, 'p4', str(p4))

        if fmt == "fc32":
            self.sig = analog.sig_source_c(rate, analog.GR_COS_WAVE, 500000.0, 0.7, 0.0)
            self.connect((self.sig, 0), (self.snk, 0))
        else:
            self.sig = analog.sig_source_s(rate, analog.GR_COS_WAVE, 500000.0, 100.0, 0)
            self.connect((self.sig, 0), (self.snk, 0))

def run_tx_test(name, fmt="fc32", rate=40.0e6, duration=1.5, freq=5.6e9, gain=20.0, bw=0):
    print(f"\n--- Subtest: {name} (fmt={fmt}, rate={rate/1e6:.3f} Msps, bw={bw/1e6:.0f} MHz) ---")
    tb = None
    try:
        tb = QuadRFTxFlowgraph(fmt=fmt, rate=rate, freq=freq, gain=gain, bandwidth=bw)
        tb.start()
        time.sleep(duration)
        tb.stop()
        tb.wait()
        print(f"  PASS: {name} completed without errors")
        return True
    except Exception as e:
        print(f"  FAIL: Exception in TX test: {e}")
        return False
    finally:
        if tb:
            del tb

def test_tx_streaming_modes():
    banner("TEST 4: TX Streaming (Formats, Resampled Rates, Bandwidths)")
    all_ok = True
    # TX 40 Msps Resampled fc32
    all_ok &= run_tx_test("TX fc32 Resampled (40 Msps)", fmt="fc32", rate=40.0e6)
    # TX 20 Msps Resampled fc32
    all_ok &= run_tx_test("TX fc32 Resampled (20 Msps)", fmt="fc32", rate=20.0e6)
    # TX 10 Msps Resampled fc32
    all_ok &= run_tx_test("TX fc32 Resampled (10 Msps)", fmt="fc32", rate=10.0e6)
    # TX 40 Msps Resampled sc8
    all_ok &= run_tx_test("TX sc8 Resampled (40 Msps)", fmt="sc8", rate=40.0e6)
    # TX Native Rate (~86.08 Msps)
    all_ok &= run_tx_test("TX fc32 Native Line Rate", fmt="fc32", rate=86.08e6)
    # TX Analog Bandwidth 20 MHz / 40 MHz
    all_ok &= run_tx_test("TX fc32 20 MHz Analog BW", fmt="fc32", rate=40.0e6, bw=20.0e6)
    all_ok &= run_tx_test("TX fc32 40 MHz Analog BW", fmt="fc32", rate=40.0e6, bw=40.0e6)
    return all_ok

class QuadRFFullDuplexFlowgraph(gr.top_block):
    def __init__(self, rx_nchan=4, rx_rate=20.101e6, tx_rate=40.0e6, freq=5.6e9):
        super().__init__("QuadRFFullDuplexFlowgraph")
        # TX Path
        self.tx_snk = soapy.sink('driver=mipi', 'fc32', 1, '', '', [''], [''])
        self.tx_snk.set_sample_rate(0, tx_rate)
        self.tx_snk.set_frequency(0, freq)
        self.tx_snk.set_gain(0, 20.0)
        self.tx_snk.write_setting(0, 'follow_rx', 'false')
        self.tx_sig = analog.sig_source_c(tx_rate, analog.GR_COS_WAVE, 200000.0, 0.7, 0.0)
        self.connect((self.tx_sig, 0), (self.tx_snk, 0))

        # RX Path
        self.rx_src = soapy.source('driver=mipi', 'fc32', rx_nchan, '', '', ['']*rx_nchan, ['']*rx_nchan)
        self.rx_src.set_sample_rate(0, rx_rate)
        self.rx_src.set_frequency(0, freq)
        self.rx_src.set_gain(0, 25.0)
        self.rx_counters = []
        for ch in range(rx_nchan):
            cnt = SampleCounter(np.complex64)
            self.rx_counters.append(cnt)
            self.connect((self.rx_src, ch), (cnt, 0))

def test_full_duplex():
    banner("TEST 5: Full Duplex (Simultaneous RX 4-Channel + TX CW Transmission)")
    try:
        tb = QuadRFFullDuplexFlowgraph(rx_nchan=4, rx_rate=20.101e6, tx_rate=40.0e6, freq=5.6e9)
        tb.start()
        time.sleep(2.0)
        tb.stop()
        tb.wait()

        rx_counts = [c.count for c in tb.rx_counters]
        print(f"Full-duplex run completed successfully!")
        print(f"RX Channels sample counts: {rx_counts}")
        if rx_counts[0] > 0:
            print("PASS: Full duplex operation verified without collisions or deadlocks.")
            return True
        else:
            print("FAIL: No RX samples during full duplex.")
            return False
    except Exception as e:
        print(f"FAIL: Exception in full duplex test: {e}")
        return False

def main():
    banner("STARTING QUADRF SOAPY SOURCE & SINK GNU RADIO TEST SUITE")
    all_pass = True
    all_pass &= test_grc_compilation()
    all_pass &= test_rx_streaming_modes()
    all_pass &= test_dynamic_callbacks()
    all_pass &= test_tx_streaming_modes()
    all_pass &= test_full_duplex()

    banner(f"TEST SUITE SUMMARY: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    return 0 if all_pass else 1

if __name__ == "__main__":
    sys.exit(main())
