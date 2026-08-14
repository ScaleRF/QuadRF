// config.h
// Configuration macros and constants for CSI sweep visualization

#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------------------------------------
// Device Configuration
// ------------------------------------------------------------

#define DEVICE_PATH      "/dev/csi_stream0"
#define SDR_JTAG_CLI     "/usr/bin/quadrf-jtag"

// ------------------------------------------------------------
// FFT and Channel Configuration
// ------------------------------------------------------------

#define FFT_SIZE         4096
#define CHANNELS_IN      4
#define CHANNELS_USED    4
#define ANT_OMIT_INDEX   (-1) // use all 4 antennas/channels
#define BYTES_PER_IQ     2
#define BYTES_PER_FRAME  (CHANNELS_IN * BYTES_PER_IQ)
#define BLOCK_BYTES      (FFT_SIZE * BYTES_PER_FRAME)

// ------------------------------------------------------------
// Sweep Parameters (MHz)
// ------------------------------------------------------------

#define LO_START_MHZ     4900.0
#define LO_END_MHZ       6100.0
#define LO_STEP_MHZ      18.0
#define FS_MHZ           18.0

// ------------------------------------------------------------
// Ring Buffer Configuration
// ------------------------------------------------------------

#define MAX_QUEUED_BLOCKS   2
#define WAIT_SPIN_ITERS     2000
#define WAIT_SLEEP_US       20

// ------------------------------------------------------------
// Antenna / Sphere Projection
// ------------------------------------------------------------

// Physical antenna nearest-neighbor spacing (meters)
#define ANTENNA_SPACING_MM      45.5f
#define ANTENNA_SPACING_M       (ANTENNA_SPACING_MM * 1e-3f)

// d/lambda = d_m * f_mhz / c, where c = 299.792458 m/us
#define D_LAMBDA_PER_MHZ        (ANTENNA_SPACING_M / 299.792458f)

// 2*pi * d/lambda at a given LO frequency in MHz
#define SCALE_FACTOR_AT_MHZ(f)  (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))

// Optional rotation of the antenna coordinate system (degrees)
#define CANVAS_ROTATE_DEG   (0.0f)

// ------------------------------------------------------------
// Point Buffer Configuration
// ------------------------------------------------------------

// Max LO steps in a single sweep (generous upper bound)
#define MAX_LO_STEPS        128

// Cap points per LO deterministically (top-K by magnitude metric)
#define TOPK_PER_LO         512

// Max points the worker can emit per sweep frame
#define MAX_POINTS_PER_FRAME (MAX_LO_STEPS * TOPK_PER_LO)

// How many sweep frames of point history the renderer keeps
#define POINT_HISTORY_FRAMES 8

// Max total points in the render history buffer
#define MAX_RENDER_POINTS    (MAX_POINTS_PER_FRAME * POINT_HISTORY_FRAMES)

// ------------------------------------------------------------
// Shutter Mode (long-exposure streaming)
// ------------------------------------------------------------

// Legacy: previously the cap on the in-RAM accumulator. The accumulator
// has been replaced by a streaming writer (shutter_stream.c), so this
// no longer allocates anything. Kept around as a reference value.
#define SHUTTER_MAX_POINTS                      5000000

// Display bin grid resolution (one side; total cells = RES * RES).
#define SHUTTER_GRID_RES                        256

// Half-range of (gx, gy) covered by the display grid.
// sf_center at 6.1 GHz is ~5.8; 10 gives generous headroom.
#define SHUTTER_GRID_HALFRANGE                  10.0f

// How often the binned display buffer is rebuilt + reuploaded to the GPU.
#define SHUTTER_DISPLAY_REBUILD_EVERY_N_FRAMES  5

// Size of the in-RAM SPSC ring buffer between the producer (sphere_shutter_add)
// and the background writer thread. 32 MB / 24 B = ~1.4M points buffered.
// Tune up if you see "DROPS" on the shutter button on your storage device.
#define SHUTTER_STREAM_RING_BYTES               (32 * 1024 * 1024)

// Target size of each fwrite() the writer thread issues. Bigger = fewer
// syscalls and better sequential throughput on flash, but more memory
// pressure briefly held during one write.
#define SHUTTER_STREAM_CHUNK_BYTES              (2 * 1024 * 1024)

// ------------------------------------------------------------
// 3D Visualization
// ------------------------------------------------------------

#define WIN_WIDTH        1024
#define WIN_HEIGHT       1024

// Hemisphere mesh tessellation
#define SPHERE_LON_SEGS  128
#define SPHERE_LAT_SEGS  64

// Point cloud rendering
#define POINT_RADIUS_OFFSET 1.005f  // slight lift above sphere surface
#define GL_POINT_SIZE_DEFAULT 3.0f

// Decay per render frame (multiplied into point intensity each frame)
#define DECAY_FACTOR        0.92f

// Minimum intensity before a point is pruned from history
#define DECAY_THRESHOLD     0.01f

// GL point size range
#define POINT_SIZE_MIN      1.0f
#define POINT_SIZE_MAX      8.0f

// Point intensity gain (scales raw intensity for visibility)
#define POINT_GAIN          2.0f

// Shutter-mode live preview rendering. Only used when shutter is active;
// normal point rendering is unaffected.
#define SHUTTER_POINT_SIZE       28.0f   // base pixel size of binned splats
#define SHUTTER_POINT_SIZE_FLOOR 0.45f   // min fraction of size for low-count bins
#define SHUTTER_GAUSS_SIGMA      0.22f   // splat sigma in [0, 0.5] point-sprite coords
#define SHUTTER_MIN_ALPHA        0.5f    // alpha floor so single-sample bins are visible
// Naka-Rushton soft-saturation knob applied to intensity_sum per bin in
// sphere_shutter_rebuild_display. Smaller K = faster saturation (less
// dynamic range visible); larger K = slower saturation (more growth
// visible over time). intensity_out = I_sum / (I_sum + K).
#define SHUTTER_SATURATION_K     1.0f

// ------------------------------------------------------------
// FOV Mirroring (reciprocal lattice aliasing)
// ------------------------------------------------------------

// Half-range for lattice search (n1,n2 in {-N..N}), matches shader
#define MIRROR_SEARCH_RANGE     3

// (Mirror copies are now rendered on the GPU via lattice-offset draw calls,
//  so no CPU-side mirror buffer is needed.)

// ------------------------------------------------------------
// Frequency Range (for color mapping)
// ------------------------------------------------------------

#define FREQ_MIN_MHZ          4900.0
#define FREQ_MAX_MHZ          6100.0

// ------------------------------------------------------------
// Telemetry Configuration
// ------------------------------------------------------------

#define TELEMETRY_PRINT_EVERY_N_FRAMES  1

#endif // CONFIG_H
