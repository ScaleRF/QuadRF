// shutter_stream.h
//
// Streams shutter-mode render_point_t records into an .rfpic file on flash
// through a small in-RAM SPSC ring buffer + background writer thread.
// Exposure time is then bounded only by free disk space, not RAM.
//
// File layout written: standard rfpic v2 (header + raw render_point_t records
// + trailing NOTES block). While recording the file is named
//   ~/Desktop/rf_pics/rfpic_YYYYMMDD_HHMMSS.rfpic.partial
// and gets renamed to the final ".rfpic" name on a clean stop.

#ifndef SHUTTER_STREAM_H
#define SHUTTER_STREAM_H

#include <stdint.h>
#include "sphere_render.h"
#include "snapshot.h"

#ifdef __cplusplus
extern "C" {
#endif

// Open the output file, write a placeholder v2 header (point_count = 0),
// and start the background writer thread.
//
// path_out (if non-NULL) is filled with the FINAL ".rfpic" path that the
// .partial file will be renamed to on shutter_stream_end. The caller can
// remember this for later display / label-dialog use.
//
// Returns 0 on success, -1 on error.
int shutter_stream_begin(double lo_start_mhz, double lo_end_mhz,
                          int camera_mode,
                          char *path_out, int path_out_len);

// Push n render_point_t records into the ring. Cheap; safe to call from
// the producer thread only. If the ring is full, the points that don't
// fit are dropped and *dropped_out (if non-NULL) is set to 1.
void shutter_stream_push(const render_point_t *pts, int n, int *dropped_out);

// Signal the writer to drain, then patch the header with the final
// point_count and IMU metadata, append the NOTES block, close the file,
// verify the magic, and rename .partial → final .rfpic.
//
// Returns 0 on success, -1 on I/O error, -2 on magic verification
// failure. If count_out is non-NULL it receives the number of points
// actually written to disk.
int shutter_stream_end(const snapshot_orientation_t *orient,
                        uint32_t *count_out);

// True (non-zero) if a stream is currently active.
int shutter_stream_active(void);

// Cumulative point counter (records pushed into the ring; does not
// include drops). Cheap and lock-free.
uint64_t shutter_stream_total_pushed(void);

// Cumulative count of points dropped due to ring-full backpressure.
uint64_t shutter_stream_total_dropped(void);

// Bytes confirmed written to disk so far (total_written * sizeof(render_point_t)).
uint64_t shutter_stream_bytes_written(void);

#ifdef __cplusplus
}
#endif

#endif // SHUTTER_STREAM_H
