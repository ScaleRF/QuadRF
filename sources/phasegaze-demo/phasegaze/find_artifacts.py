import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import binned_statistic_2d
import json
import os

def process_artifacts(csv_file='calibration_points.csv'):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found.")
        return

    df = pd.read_csv(csv_file)
    if len(df) == 0:
        print("Error: CSV is empty.")
        return

    # Only keep points above threshold — artifacts are persistently high
    INTENSITY_THRESHOLD = 0.1
    df = df[df['intensity'] > INTENSITY_THRESHOLD].reset_index(drop=True)
    if len(df) == 0:
        print("No points above threshold.")
        return

    # Split chronologically (first half = train, second half = test)
    split_idx = len(df) // 2
    train_df = df.iloc[:split_idx]
    test_df  = df.iloc[split_idx:]

    d_lambda     = 0.85
    scale_factor = 2.0 * np.pi * d_lambda

    u_train   = train_df['gx'].values / scale_factor
    v_train   = train_df['gy'].values / scale_factor
    int_train = train_df['intensity'].values

    u_test   = test_df['gx'].values / scale_factor
    v_test   = test_df['gy'].values / scale_factor
    int_test = test_df['intensity'].values

    best_performance = float('inf')
    best_bins        = 0
    stable_artifacts = []

    bin_sizes = [100, 200, 500, 1000, 2000, 5000, 10000]

    for bins in bin_sizes:
        print(f"Testing {bins}x{bins} bins...")

        # 1. Sum all above-threshold intensities into each bin over the full
        #    training half, then normalise to [0,1] so bin size doesn't affect
        #    the threshold.
        stat_train, x_edge, y_edge, _ = binned_statistic_2d(
            u_train, v_train, int_train,
            statistic='sum',
            bins=bins,
            range=[[-1, 1], [-1, 1]]
        )
        stat_train = np.nan_to_num(stat_train)

        # Normalise to [0,1]
        train_max = stat_train.max()
        if train_max > 0:
            stat_train_norm = stat_train / train_max
        else:
            print("  All-zero training grid, skipping.")
            del stat_train
            continue

        # Hot bins are those that accumulate well above the field average.
        # 3-sigma on the normalised non-zero bins.
        valid = stat_train_norm[stat_train_norm > 0]
        threshold = np.mean(valid) + 3.0 * np.std(valid)
        mask = stat_train_norm > threshold
        n_hot = int(mask.sum())
        print(f"  Threshold: {threshold:.4f}  Hot bins: {n_hot}")

        del stat_train, stat_train_norm

        # 2. Validate on the test half using the same sum+normalise approach.
        stat_test, _, _, _ = binned_statistic_2d(
            u_test, v_test, int_test,
            statistic='sum',
            bins=[x_edge, y_edge]
        )
        stat_test = np.nan_to_num(stat_test)

        test_max = stat_test.max()
        if test_max > 0:
            stat_test_norm = stat_test / test_max
        else:
            stat_test_norm = stat_test.copy()

        del stat_test

        masked_test = stat_test_norm.copy()
        masked_test[mask] = 0.0

        # Lower remaining energy = mask is working well
        performance = float(np.sum(masked_test))
        print(f"  Performance (residual energy): {performance:.4f}")

        # Plot for sizes that fit in memory
        if bins <= 1000:
            fig, axes = plt.subplots(1, 3, figsize=(15, 5))
            fig.suptitle(f'Bin size {bins}x{bins}', fontsize=12)

            im0 = axes[0].imshow(stat_test_norm.T, origin='lower',
                                  extent=[-1, 1, -1, 1], cmap='viridis',
                                  vmin=0, vmax=1)
            axes[0].set_title('Before Mask (normalised sum)')
            plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

            im1 = axes[1].imshow(mask.T, origin='lower',
                                  extent=[-1, 1, -1, 1], cmap='gray')
            axes[1].set_title('Mask')

            im2 = axes[2].imshow(masked_test.T, origin='lower',
                                  extent=[-1, 1, -1, 1], cmap='viridis',
                                  vmin=0, vmax=1)
            axes[2].set_title('After Mask')
            plt.colorbar(im2, ax=axes[2], fraction=0.046, pad=0.04)

            plt.tight_layout()
            plt.savefig(f'mask_iteration_{bins}.png')
            plt.close()
        else:
            print(f"  Skipping plot for {bins}x{bins} (too large to render).")

        del stat_test_norm, masked_test

        if performance < best_performance:
            best_performance = performance
            best_bins        = bins

            stable_artifacts = []
            for i in range(bins):
                for j in range(bins):
                    if mask[i, j]:
                        u_c = (x_edge[i] + x_edge[i + 1]) / 2.0
                        v_c = (y_edge[j] + y_edge[j + 1]) / 2.0
                        stable_artifacts.append({'u': float(u_c), 'v': float(v_c)})
        else:
            print(f"  Performance degraded vs best ({best_bins}x{best_bins}), continuing.")

        del mask

    print(f"\nOptimal bin size: {best_bins}x{best_bins}")
    print(f"Artifact bins in mask: {len(stable_artifacts)}")

    with open('calibration_mask.json', 'w') as f:
        json.dump(stable_artifacts, f, indent=4)
    print("Saved calibration_mask.json")

if __name__ == "__main__":
    process_artifacts()
