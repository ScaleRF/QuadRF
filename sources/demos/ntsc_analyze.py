#!/usr/bin/env python3
"""Offline DSP checks for quadrf-ntsc-demod dumps.

Tune first (R5):

  quadrf-jtag --rx autosteer=1,antennas=15,interleave=0,tone_en=0,bw=12.0,agc=-14.0,freq=5806

Decoded stills (not consumed here):

  mkdir -p /tmp/ntsc-cap
  ./quadrf-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 \
      --dump_ppm /tmp/ntsc-cap --dump_after 8 --dump_count 6 --duration 12

IQ / FM / locked lines for this script:

  ./quadrf-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 \
      --dump_iq /tmp/ntsc.cf32 --dump_fm /tmp/ntsc.f32 --dump_lines /tmp/ntsc-lines.f32 \
      --dump_stop

  python3 ntsc_analyze.py iq  dump.cf32 [--fs-iq 28636360] [--dev 3.5e6] [--expect-ire IRE]
  python3 ntsc_analyze.py fm  dump.f32  [--fs-iq 28636360] [--dev 3.5e6] [--expect-ire IRE]
  python3 ntsc_analyze.py lines dump.f32 [--fs-vid 14318180] [--dev 3.5e6]

FM dumps are float32 discriminator samples AFTER 2:1 halfband. Each value is
still radians of IQ-phase change per IQ sample (the decimator is a lowpass +
keep-even, it does not rescale). Instantaneous frequency:

    f_hz = x * fs_iq / (2π)

Expected DC tone for a constant IRE with deviation `dev` Hz at 100 IRE:

    f_expect = (IRE / 100) * dev
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

FSC = 3_579_545.0
FS_IQ = 8.0 * FSC
FS_VID = 4.0 * FSC
H_LINE = 63.555e-6
DEV_DEFAULT = 3.5e6


def load_f32(path: str) -> np.ndarray:
    x = np.fromfile(path, dtype=np.float32)
    if x.size == 0:
        raise SystemExit(f"empty dump: {path}")
    return x


def load_cf32(path: str) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 2:
        raise SystemExit(f"empty IQ dump: {path}")
    return raw[0::2] + 1j * raw[1::2]


def pct(x: np.ndarray, p) -> float:
    return float(np.percentile(x, p))


def tone_hz_from_iq(iq: np.ndarray, fs: float) -> tuple[float, float, float]:
    """FFT peak frequency, peak/mean power (dB), 99% occupied BW (Hz)."""
    n = int(2 ** math.floor(math.log2(iq.size)))
    x = iq[:n] * np.hanning(n)
    spec = np.fft.fftshift(np.fft.fft(x))
    p = np.abs(spec) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs))
    k = int(np.argmax(p))
    peak_hz = float(freqs[k])
    peak_db = 10.0 * math.log10(float(p[k]) / (float(np.mean(p)) + 1e-30))
    c = np.cumsum(p)
    tot = float(c[-1])
    lo = int(np.searchsorted(c, 0.005 * tot))
    hi = int(np.searchsorted(c, 0.995 * tot))
    bw99 = float(abs(freqs[min(hi, n - 1)] - freqs[max(lo, 0)]))
    return peak_hz, peak_db, bw99


def autocorr_lag(x: np.ndarray, fs: float, expect_s: float, search=0.15) -> tuple[float, float]:
    """Parabolic peak of |Rxx| near expect_s. Returns (period_s, peak)."""
    n = min(x.size, 1 << 18)
    y = x[:n] - float(np.mean(x[:n]))
    spec = np.fft.rfft(y, n=1 << int(math.ceil(math.log2(2 * n))))
    r = np.fft.irfft(spec * np.conj(spec))
    r = r[:n]
    lo = max(1, int((expect_s * (1.0 - search)) * fs))
    hi = min(n - 2, int((expect_s * (1.0 + search)) * fs))
    if hi <= lo + 2:
        return float("nan"), float("nan")
    k = lo + int(np.argmax(r[lo:hi]))
    # parabolic interpolate
    denom = r[k - 1] - 2.0 * r[k] + r[k + 1]
    frac = 0.5 * (r[k - 1] - r[k + 1]) / denom if abs(denom) > 1e-20 else 0.0
    lag = k + frac
    return lag / fs, float(r[k] / (r[0] + 1e-30))


def analyze_iq(path: str, fs_iq: float, dev: float, expect_ire: float | None) -> int:
    iq = load_cf32(path)
    mag = np.abs(iq)
    i_m, q_m = float(np.mean(iq.real)), float(np.mean(iq.imag))
    rms = float(np.sqrt(np.mean(mag * mag)))
    peak_hz, peak_db, bw99 = tone_hz_from_iq(iq, fs_iq)

    print(f"IQ  n={iq.size}  T={iq.size / fs_iq * 1e3:.2f} ms  fs={fs_iq / 1e6:.6f} Msps")
    print(f"    |z| mean={float(np.mean(mag)):.4f}  rms={rms:.4f}  min={float(np.min(mag)):.4f}")
    print(f"    DC I={i_m:+.4f}  Q={q_m:+.4f}  (should be ~0 for CS8 FM)")
    print(f"    FFT peak={peak_hz / 1e6:+.4f} MHz  peak/mean={peak_db:.1f} dB  BW99={bw99 / 1e3:.1f} kHz")

    rc = 0
    if expect_ire is not None:
        want = (expect_ire / 100.0) * dev
        err = peak_hz - want
        print(f"    expect IRE={expect_ire:g} → {want / 1e6:+.4f} MHz  err={err / 1e3:+.1f} kHz")
        # DC/blank: allow 80 kHz LO residual. Other IRE: 8% of commanded deviation.
        tol = 80e3 if abs(expect_ire) < 1.0 else max(80e3, 0.08 * abs(want))
        if abs(err) > tol:
            print(f"    FAIL  |err| > {tol / 1e3:.0f} kHz")
            rc = 1
        else:
            print(f"    PASS  |err| <= {tol / 1e3:.0f} kHz")
        if abs(expect_ire) >= 1.0 and abs(peak_hz) < 50e3:
            print("    FAIL  tone at DC — polarity/scale collapsed or TX not FM")
            rc = 1
    if bw99 > 250e3 and expect_ire is not None:
        print(f"    NOTE  occupied BW {bw99 / 1e3:.0f} kHz; a constant-IRE FM should be a tone")
    return rc


def analyze_fm(path: str, fs_iq: float, dev: float, expect_ire: float | None) -> int:
    x = load_f32(path)
    # Discriminator is rad / IQ-sample; decimated stream still uses fs_iq for Hz.
    hz = x * (fs_iq / (2.0 * math.pi))
    mu, sd = float(np.mean(hz)), float(np.std(hz))
    print(f"FM  n={x.size}  T={x.size / (fs_iq / 2) * 1e3:.2f} ms  (vid rate {fs_iq / 2 / 1e6:.6f} Msps)")
    print(f"    mean={mu / 1e6:+.4f} MHz  std={sd / 1e3:.2f} kHz  "
          f"p1={pct(hz, 1) / 1e6:+.3f}  p50={pct(hz, 50) / 1e6:+.3f}  p99={pct(hz, 99) / 1e6:+.3f}")
    print(f"    disc rad/samp mean={float(np.mean(x)):+.5f}  (π would mean wrap/clip)")

    period, ac = autocorr_lag(hz, fs_iq / 2.0, H_LINE)
    if math.isfinite(period):
        print(f"    line autocorr T={period * 1e6:.3f} µs  (NTSC 63.555 µs)  R={ac:.3f}")

    rc = 0
    if expect_ire is not None:
        want = (expect_ire / 100.0) * dev
        err = mu - want
        print(f"    expect IRE={expect_ire:g} → {want / 1e6:+.4f} MHz  err={err / 1e3:+.1f} kHz")
        tol = 80e3 if abs(expect_ire) < 1.0 else max(80e3, 0.08 * abs(want))
        if abs(err) > tol:
            print(f"    FAIL  |err| > {tol / 1e3:.0f} kHz")
            rc = 1
        else:
            print(f"    PASS  |err| <= {tol / 1e3:.0f} kHz")
    return rc


def analyze_lines(path: str, fs_vid: float, dev: float) -> int:
    x = load_f32(path)
    spl = int(round(fs_vid * H_LINE))
    nlines = x.size // spl
    if nlines < 1:
        raise SystemExit(f"need at least one {spl}-sample line, got {x.size}")
    lines = x[: nlines * spl].reshape(nlines, spl)
    t_us = np.arange(spl) / fs_vid * 1e6

    # x is rad / IQ-sample; f_hz = x * (2 fs_vid) / 2π; IRE = 100 * f_hz / dev
    ire = lines * (2.0 * fs_vid) * 100.0 / (2.0 * math.pi * dev)

    sync_n = int(round(fs_vid * 4.7e-6))
    burst0 = sync_n + int(round(fs_vid * 0.7e-6))
    burst1 = burst0 + int(round(fs_vid * 2.5e-6))
    act0 = sync_n + int(round(fs_vid * 4.7e-6))
    act1 = act0 + int(round(fs_vid * 52.6e-6))

    sync_ire = float(np.mean(ire[:, :sync_n]))
    blank_ire = float(np.mean(ire[:, sync_n:burst0]))
    burst = ire[:, burst0:burst1] - blank_ire
    # 9-cycle -U burst at 4*fSC: amplitude ~ 20 IRE, period 4 samples
    burst_pk = float(np.sqrt(2.0) * np.std(burst))
    active = ire[:, act0:act1]

    print(f"LINES  {nlines} x {spl}  fs_vid={fs_vid / 1e6:.6f} Msps")
    print(f"    t[0..end] = {t_us[0]:.2f} .. {t_us[-1]:.2f} µs  (line 63.555 µs)")
    print(f"    sync  IRE={sync_ire:+.2f}  (expect −40)")
    print(f"    blank IRE={blank_ire:+.2f}  (expect 0, back-porch before burst)")
    print(f"    burst RMS*√2={burst_pk:.2f} IRE  (expect ~20)")
    print(f"    active IRE p5={pct(active.ravel(), 5):+.1f}  p50={pct(active.ravel(), 50):+.1f}  "
          f"p95={pct(active.ravel(), 95):+.1f}")

    # H-sync width: run of samples below (sync+blank)/2 near the start
    thr = 0.5 * (sync_ire + blank_ire)
    widths = []
    for ln in ire:
        below = ln[: spl // 4] < thr
        # first run
        w, run = 0, 0
        for b in below:
            if b:
                run += 1
                w = max(w, run)
            else:
                if w:
                    break
                run = 0
        widths.append(w / fs_vid * 1e6)
    print(f"    H width mean={float(np.mean(widths)):.2f} µs  (expect 4.7)")

    rc = 0
    if abs(sync_ire - blank_ire) < 15:
        print("    FAIL  sync-to-blank < 15 IRE (no H, or polarity inverted)")
        rc = 1
    if sync_ire > blank_ire:
        print("    FAIL  sync above blank — FM polarity inverted vs SMPTE 170M")
        rc = 1
    if abs((sync_ire - blank_ire) + 40.0) > 15:
        print(f"    NOTE  sync-blank = {sync_ire - blank_ire:+.1f} IRE (target −40)")
    return rc


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("kind", choices=("iq", "fm", "lines"))
    p.add_argument("path")
    p.add_argument("--fs-iq", type=float, default=FS_IQ)
    p.add_argument("--fs-vid", type=float, default=FS_VID)
    p.add_argument("--dev", type=float, default=DEV_DEFAULT)
    p.add_argument("--expect-ire", type=float, default=None)
    args = p.parse_args()
    if args.kind == "iq":
        return analyze_iq(args.path, args.fs_iq, args.dev, args.expect_ire)
    if args.kind == "fm":
        return analyze_fm(args.path, args.fs_iq, args.dev, args.expect_ire)
    return analyze_lines(args.path, args.fs_vid, args.dev)


if __name__ == "__main__":
    sys.exit(main())
