// worker.c
// Worker thread: CSI sweep, FFT, phase delta computation, sphere point output

#include "worker.h"
#include "config.h"
#include "utils.h"
#include "ring_buffer.h"
#include "signal_processing.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

void *worker(void *arg)
{
    ctx_t *c = (ctx_t*)arg;

    uint8_t *blk = (uint8_t*)malloc(BLOCK_BYTES);
    if (!blk) die("malloc blk");

    float Vraw[FFT_SIZE];

    const int half = FFT_SIZE / 2;

    double lo_start = LO_START_MHZ;
    double lo_end = LO_END_MHZ;

    pthread_mutex_lock(&c->range_mtx);
    lo_start = c->lo_start_mhz;
    lo_end = c->lo_end_mhz;
    pthread_mutex_unlock(&c->range_mtx);

    int nsteps = (int)ceil((lo_end - lo_start) / LO_STEP_MHZ);
    double *lo_list = (double*)malloc((size_t)nsteps * sizeof(double));
    if (!lo_list) die("malloc lo_list");
    for (int i = 0; i < nsteps; ++i)
        lo_list[i] = lo_start + (double)i * LO_STEP_MHZ;

    heap_item_t topk[TOPK_PER_LO];

    uint64_t frame_idx = 0;

    float rot_rad = CANVAS_ROTATE_DEG * (float)M_PI / 180.0f;
    float rot_cs = cosf(rot_rad);
    float rot_sn = sinf(rot_rad);

    // scale_factor is computed per bin from the actual RF frequency below

    int prev_calibrate = 0;

    while (!c->quit)
    {
        // Detect calibration mode transitions
        int cal = c->calibrate_mode;
        if (cal && !prev_calibrate) {
            spur_mask_cal_start(&c->mask, nsteps);
        } else if (!cal && prev_calibrate) {
            spur_mask_finalize(&c->mask);
        }
        prev_calibrate = cal;

        if (c->range_changed)
        {
            pthread_mutex_lock(&c->range_mtx);
            lo_start = c->lo_start_new;
            lo_end = c->lo_end_new;
            c->lo_start_mhz = lo_start;
            c->lo_end_mhz = lo_end;
            c->range_changed = 0;
            pthread_mutex_unlock(&c->range_mtx);

            free(lo_list);
            nsteps = (int)ceil((lo_end - lo_start) / LO_STEP_MHZ);
            if (nsteps < 1) nsteps = 1;
            lo_list = (double*)malloc((size_t)nsteps * sizeof(double));
            if (!lo_list) die("malloc lo_list");
            for (int i = 0; i < nsteps; ++i)
                lo_list[i] = lo_start + (double)i * LO_STEP_MHZ;

            // LO indices changed — any in-progress calibration is invalid
            if (cal)
                spur_mask_cal_start(&c->mask, nsteps);
        }

        uint64_t t_frame0 = now_ns();

        uint64_t ns_read = 0, ns_lo = 0, ns_fft = 0, ns_proc = 0;
        uint32_t blocks = 0;
        int npoints = 0;

        int cam = c->camera_mode;
        float sf_center = SCALE_FACTOR_AT_MHZ((float)(lo_start + lo_end) * 0.5f);

        // Pull current calibration values once per frame; worker doesn't need
        // them locked atomically (single-writer, periodic reader is fine).
        float cal_eps10 = 0.0f, cal_eps20 = 0.0f, cal_eps30 = 0.0f;
        float cal_m10 = 0.0f, cal_b10 = 0.0f;
        float cal_m20 = 0.0f, cal_b20 = 0.0f;
        float cal_m30 = 0.0f, cal_b30 = 0.0f;
        int cal_delay_fitted = 0;
        int cal_capture_active = 0;
        int cal_legacy_loaded = 0;
        if (c->phase_cal) {
            cal_capture_active = c->phase_cal->capture_active;
            if (!cal_capture_active && c->phase_cal->loaded) {
                if (c->phase_cal->delay_cal.fitted) {
                    cal_delay_fitted = 1;
                    cal_m10 = c->phase_cal->delay_cal.m10;
                    cal_b10 = c->phase_cal->delay_cal.b10;
                    cal_m20 = c->phase_cal->delay_cal.m20;
                    cal_b20 = c->phase_cal->delay_cal.b20;
                    cal_m30 = c->phase_cal->delay_cal.m30;
                    cal_b30 = c->phase_cal->delay_cal.b30;
                } else {
                    cal_legacy_loaded = 1;
                    cal_eps10 = c->phase_cal->eps10_rad;
                    cal_eps20 = c->phase_cal->eps20_rad;
                    cal_eps30 = c->phase_cal->eps30_rad;
                }
            }
        }

        // Intensity-weighted centroid accumulators for the boresight overlay.
        double cent_gx_sum = 0.0;
        double cent_gy_sum = 0.0;
        double cent_w_sum  = 0.0;

        read_telem_t rt = (read_telem_t){0};

        // Pipeline: program LO0 before reading first block
        double lo_curr = lo_list[0];
        uint64_t t0 = now_ns();
        (void)program_set_freq(c->fd, lo_curr);
        ns_lo += (now_ns() - t0);

        for (int idx = 0; idx < nsteps && !c->quit; ++idx)
        {
            if (c->range_changed)
                break;

            lo_curr = lo_list[idx];
            double lo_next = (idx + 1 < nsteps) ? lo_list[idx + 1] : lo_list[0];

            uint64_t tr0 = now_ns();
            ring_wait_and_get_one_block_fast(c->fd, c->ring, c->ring_size, blk, &rt);
            ns_read += (now_ns() - tr0);

            uint64_t tl0 = now_ns();
            (void)program_set_freq(c->fd, lo_next);
            ns_lo += (now_ns() - tl0);

            blocks++;

            uint64_t tf0 = now_ns();
            for (int u = 0; u < CHANNELS_USED; ++u)
            {
                int src_ch = u;
                cs8_to_fftw_ch((const int8_t*)blk, src_ch, c->fft_in);
                fftwf_execute(c->plan[u]);
            }
            ns_fft += (now_ns() - tf0);

            uint64_t tp0 = now_ns();

            float vmax = 1e-9f;
            for (int k = 0; k < FFT_SIZE; ++k)
            {
                int i = (k + half) % FFT_SIZE;

                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                float s0 = re0*re0 + im0*im0;
                float s1 = re1*re1 + im1*im1;
                float s2 = re2*re2 + im2*im2;
                float s3 = re3*re3 + im3*im3;

                float v = logf(1.0f + (s0 + s1 + s2 + s3));
                Vraw[k] = v;
                if (v > vmax) vmax = v;
            }
            if (vmax < 1e-9f) vmax = 1e-9f;

            int hsz = 0;
            for (int k = 0; k < FFT_SIZE; ++k)
                hsz = heap_push_topk(topk, hsz, TOPK_PER_LO, Vraw[k], k);

            if (cal)
                spur_mask_cal_accumulate(&c->mask, idx, topk, hsz, Vraw, vmax);

            // bias_ij values come from the phase-offset calibration. They
            // are zero when no cal is loaded, which reproduces the prior
            // behaviour exactly.
            float bias10 = cal_eps10;
            float bias20 = cal_eps20;
            float bias30 = cal_eps30;

            for (int t = 0; t < hsz; ++t)
            {
                if (npoints >= MAX_POINTS_PER_FRAME) break;

                int k = topk[t].k;
                float v = Vraw[k] / vmax;
                v = clampf(v, 0.0f, 1.0f);

                // Subtract spur mask (before expensive phase computation)
                if (!cal && c->mask.active) {
                    v -= spur_mask_value(&c->mask, idx, k);
                    if (v <= 0.0f) continue;
                }

                int i = (k + half) % FFT_SIZE;

                // RF frequency for this bin — used for scale, color, and cal.
                double rf = lo_curr + FS_MHZ * ((double)k - (double)half)
                            / (double)FFT_SIZE;

                if (cal_delay_fitted) {
                    float f = (float)rf;
                    bias10 = cal_m10 * f + cal_b10;
                    bias20 = cal_m20 * f + cal_b20;
                    bias30 = cal_m30 * f + cal_b30;
                } else if (!cal_legacy_loaded) {
                    bias10 = 0.0f;
                    bias20 = 0.0f;
                    bias30 = 0.0f;
                }

                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                float cre10 = re1*re0 + im1*im0;
                float cim10 = im1*re0 - re1*im0;
                float phi10 = atan2f(cim10, cre10);

                float cre23 = re2*re3 + im2*im3;
                float cim23 = im2*re3 - re2*im3;
                float phi23 = atan2f(cim23, cre23);

                float cre20 = re2*re0 + im2*im0;
                float cim20 = im2*re0 - re2*im0;
                float phi20 = atan2f(cim20, cre20);

                float cre30 = re3*re0 + im3*im0;
                float cim30 = im3*re0 - re3*im0;
                float phi30 = atan2f(cim30, cre30);

                float cre21 = re2*re1 + im2*im1;
                float cim21 = im2*re1 - re2*im1;
                float phi21 = atan2f(cim21, cre21);

                phi10 = wrap_pi_f(phi10 - bias10);
                phi23 = wrap_pi_f(phi23 - (bias20 - bias30));
                phi20 = wrap_pi_f(phi20 - bias20);
                phi30 = wrap_pi_f(phi30 - bias30);
                phi21 = wrap_pi_f(phi21 - (bias20 - bias10));

                phi10 = circular_mean2_f(phi10, phi23);
                phi30 = circular_mean2_f(phi30, phi21);

                float best_gx = 0.0f, best_gy = 0.0f;
                {
                    const float inv_sqrt3 = (float)(1.0 / 1.7320508075688772);
                    const float reg = 1e-3f;
                    float best_cost = 1e30f;

                    for (int n10 = -2; n10 <= 2; ++n10)
                    for (int n20 = -2; n20 <= 2; ++n20)
                    for (int n30 = -2; n30 <= 2; ++n30)
                    {
                        float y1 = phi10 + (float)(2.0 * M_PI) * (float)n10;
                        float y2 = phi20 + (float)(2.0 * M_PI) * (float)n20;
                        float y3 = phi30 + (float)(2.0 * M_PI) * (float)n30;

                        float gx = (y3 - y1) * inv_sqrt3;
                        float gy = (y1 + 2.0f*y2 + y3) * (1.0f / 3.0f);

                        float p1 = -0.8660254037844386f * gx + 0.5f * gy;
                        float p2 = gy;
                        float p3 =  0.8660254037844386f * gx + 0.5f * gy;

                        float r1 = y1 - p1;
                        float r2 = y2 - p2;
                        float r3 = y3 - p3;
                        float cost = r1*r1 + r2*r2 + r3*r3 + reg * (gx*gx + gy*gy);
                        if (cost < best_cost)
                        {
                            best_cost = cost;
                            best_gx = gx;
                            best_gy = gy;
                        }
                    }
                }

                // Apply rotation to match physical antenna orientation
                float gx_r = best_gx * rot_cs - best_gy * rot_sn;
                float gy_r = best_gx * rot_sn + best_gy * rot_cs;

                float scale_factor = SCALE_FACTOR_AT_MHZ((float)rf);

                // Check if this point falls on the hemisphere
                float u_dir = gx_r / scale_factor;
                float v_dir = gy_r / scale_factor;
                if (u_dir * u_dir + v_dir * v_dir > 1.0f)
                    continue;

                if (cam) {
                    // Test in raw gradient coords with this bin's d/lambda so
                    // neighbor exclusion matches its RF (see hemi blue/orange).
                    float d_lambda_bin = D_LAMBDA_PER_MHZ * (float)rf;
                    if (!is_deterministic(gx_r, gy_r, d_lambda_bin))
                        continue;
                    float correction = sf_center / scale_factor;
                    gx_r *= correction;
                    gy_r *= correction;
                }

                // Frequency-based color
                float hue = (float)((rf - FREQ_MIN_MHZ) / (FREQ_MAX_MHZ - FREQ_MIN_MHZ));
                hue = clampf(hue, 0.0f, 1.0f);

                float rr, gg, bb;
                hsv_to_rgb(hue, 1.0f, v, &rr, &gg, &bb);

                point_data_t *pt = &c->points_back[npoints];
                pt->gx = gx_r;
                pt->gy = gy_r;
                pt->r = rr;
                pt->g = gg;
                pt->b = bb;
                pt->intensity = v;
                npoints++;

                // Intensity-weighted (gx, gy) centroid for the boresight overlay.
                {
                    double w = (double)v;
                    cent_gx_sum += w * (double)gx_r;
                    cent_gy_sum += w * (double)gy_r;
                    cent_w_sum  += w;
                }
            }

            ns_proc += (now_ns() - tp0);
        }

        if (cal)
            spur_mask_cal_end_frame(&c->mask);

        // Publish live centroid for the boresight UI.
        if (c->phase_cal) {
            int cent_valid = 0;
            float cent_gx = 0.0f, cent_gy = 0.0f;
            if (cent_w_sum > 1e-9) {
                cent_gx = (float)(cent_gx_sum / cent_w_sum);
                cent_gy = (float)(cent_gy_sum / cent_w_sum);
                cent_valid = 1;
            }
            phase_cal_set_live_centroid(c->phase_cal, cent_gx, cent_gy,
                                        cent_valid);
        }

        // Publish completed sweep-frame: swap front/back point buffers
        pthread_mutex_lock(&c->mtx);
        point_data_t *tmp = c->points_front;
        c->points_front = c->points_back;
        c->points_back  = tmp;
        c->npoints_front = npoints;
        c->npoints_back  = 0;

        uint64_t t_frame1 = now_ns();
        double t_frame_ms = ns_to_ms(t_frame1 - t_frame0);
        double fps = (t_frame_ms > 1e-9) ? (1000.0 / t_frame_ms) : 0.0;

        c->telem.frame_idx = frame_idx;
        c->telem.t_read_ms  = ns_to_ms(ns_read);
        c->telem.t_lo_ms    = ns_to_ms(ns_lo);
        c->telem.t_fft_ms   = ns_to_ms(ns_fft);
        c->telem.t_proc_ms  = ns_to_ms(ns_proc);
        c->telem.t_frame_ms = t_frame_ms;

        c->telem.t_read_getinfo_ms  = ns_to_ms(rt.ns_getinfo);
        c->telem.t_read_copy_ms     = ns_to_ms(rt.ns_copy);
        c->telem.t_read_consume_ms  = ns_to_ms(rt.ns_consume);
        c->telem.read_calls_getinfo = rt.calls_getinfo;
        c->telem.read_calls_consume = rt.calls_consume;
        c->telem.read_wraps         = rt.wraps;

        c->telem.blocks = blocks;
        c->telem.points = (uint32_t)npoints;
        c->telem.fps = fps;
        c->telem.mask_cal_frames = c->mask.cal_frames;
        c->telem.mask_active = c->mask.active;

        pthread_mutex_unlock(&c->mtx);

        frame_idx++;

        if ((frame_idx % TELEMETRY_PRINT_EVERY_N_FRAMES) == 0)
        {
            fprintf(stdout,
                "[frame %llu] total=%.1f ms (%.2f fps)  read=%.1f  "
                "LO=%.1f  FFT=%.1f  proc=%.1f  blocks=%u  points=%d\n",
                (unsigned long long)frame_idx,
                c->telem.t_frame_ms, c->telem.fps,
                c->telem.t_read_ms,
                c->telem.t_lo_ms, c->telem.t_fft_ms, c->telem.t_proc_ms,
                blocks, npoints);
            fflush(stdout);
        }
    }

    free(lo_list);
    free(blk);
    return NULL;
}
