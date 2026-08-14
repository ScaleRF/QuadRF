#!/usr/bin/env python3
"""Plot multi-frequency delay matching calibration from phase_calibration.json."""

import argparse
import json
import os

import matplotlib.pyplot as plt
import numpy as np


def load_delay_cal(path):
    with open(path) as f:
        data = json.load(f)
    dc = data.get("delay_cal")
    if not dc:
        raise SystemExit(f"No delay_cal block in {path}")
    return dc


def plot_delay_cal(dc, out_path=None):
    baselines = [
        ("10", "m10_rad_per_mhz", "b10_rad", "eps10_rad", "C0"),
        ("20", "m20_rad_per_mhz", "b20_rad", "eps20_rad", "C1"),
        ("30", "m30_rad_per_mhz", "b30_rad", "eps30_rad", "C2"),
    ]

    samples = dc.get("samples", [])
    freqs = [s["freq_mhz"] for s in samples]
    if not freqs:
        raise SystemExit("No per-slot samples in delay_cal")

    f_min = min(freqs) - 50
    f_max = max(freqs) + 50
    f_line = np.linspace(f_min, f_max, 200)

    fig, ax = plt.subplots(figsize=(9, 5))

    for label, m_key, b_key, eps_key, color in baselines:
        m = dc[m_key]
        b = dc[b_key]
        y_line = m * f_line + b
        ax.plot(
            f_line,
            y_line,
            color=color,
            label=f"eps{label}(f) = {m:.3g}·f + {b:.3f}",
        )

        x_pts = [s["freq_mhz"] for s in samples]
        y_pts = [s[eps_key] for s in samples]
        ax.scatter(x_pts, y_pts, color=color, s=40, zorder=3)

    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Phase offset (rad)")
    ax.set_title("Linear delay matching calibration")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=150)
        print(f"Saved {out_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i",
        "--input",
        default="phase_calibration.json",
        help="Path to phase_calibration.json",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="delay_cal_plot.png",
        help="Output PNG path (omit to show interactively)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.input):
        raise SystemExit(f"Input not found: {args.input}")

    dc = load_delay_cal(args.input)
    plot_delay_cal(dc, out_path=args.output)


if __name__ == "__main__":
    main()
