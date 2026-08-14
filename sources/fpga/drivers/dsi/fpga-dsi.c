// SPDX-License-Identifier: GPL-2.0
/*
 * DSI byte-stream panel (/dev/dsi_stream0) with staging-mmap:
 * - Userspace mmaps a vmalloc_user() staging block (N frames).
 * - Userspace writes RGB bytes (already formatted) into staging[head].
 * - Userspace issues DSI_IOC_QUEUE_NEXT; flip thread copies staging->scanout FB.
 * - No RGB reordering, no float/IQ path.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/iosys-map.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/smp.h>
#include <linux/kref.h>
#include <linux/math64.h>
#include <linux/io.h>

#include <drm/drm_device.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_client.h>
#include <drm/drm_atomic.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include <video/mipi_display.h>
#if defined(__has_include)
#  if __has_include(<drm/mipi_dsi.h>)
#    include <drm/mipi_dsi.h>
#  else
#    include <drm/drm_mipi_dsi.h>
#  endif
#else
#  include <drm/mipi_dsi.h>
#endif

#ifndef no_llseek
#define no_llseek noop_llseek
#endif

#define DRV_NAME "dsi-stream-panel"
#define DSI_STREAM_NUM_BUFS  4

#ifndef DSI_IOC_MAGIC
#define DSI_IOC_MAGIC   'D'
#endif

struct dsi_fb_info {
    __u64 fb_bytes;
    __u32 fb_count;
    __u32 head;
    __u32 tail;
    __s32 queued;
    __u32 _pad;
};
#ifndef DSI_IOC_GET_FB_INFO
#define DSI_IOC_GET_FB_INFO _IOR(DSI_IOC_MAGIC, 0x10, struct dsi_fb_info)
#endif

/* Basic ioctls */
#define DSI_IOC_QUEUE_NEXT _IO(DSI_IOC_MAGIC, 0x11)

/* Userspace can read these via sysfs on /dev/dsi_stream0 device */
static ssize_t frames_in_show(struct device *d, struct device_attribute *a, char *buf);
static ssize_t frames_flipped_show(struct device *d, struct device_attribute *a, char *buf);
static ssize_t queued_show(struct device *d, struct device_attribute *a, char *buf);
static ssize_t ring_show(struct device *d, struct device_attribute *a, char *buf);
static ssize_t dsi_clock_hz_show(struct device *d, struct device_attribute *a, char *buf);
static ssize_t pixel_clock_khz_show(struct device *d, struct device_attribute *a, char *buf);
static DEVICE_ATTR_RO(frames_in);
static DEVICE_ATTR_RO(frames_flipped);
static DEVICE_ATTR_RO(queued);
static DEVICE_ATTR_RO(ring);
static DEVICE_ATTR_RO(dsi_clock_hz);
static DEVICE_ATTR_RO(pixel_clock_khz);

struct dsi_stream_dev {
	struct kref refcount;

	struct device *dev;
	struct drm_device *drm;
	struct mipi_dsi_device *dsi;
	struct drm_panel panel;
	bool prepared, enabled;

	/* Geometry */
	u32 bytes_per_line;      /* payload bytes per line userspace writes */
	u32 height;              /* lines per frame */
	u32 fps;
	u32 lanes;
	bool continuous_clk;
	u32 fb_bpp;              /* 3 for RGB888 */
	u32 target_dsi_clock_hz; /* D-PHY clock lane frequency; 0 = derive from fps */
	bool start_on_load;      /* start DRM client/DSI stream as soon as the panel binds */
	bool zero_idle_on_start; /* present all-zero payload before userspace queues data */
	u32 mode_clock_khz;      /* pixel clock requested from DRM */
	u64 dsi_clock_hz;        /* estimated D-PHY clock lane frequency */
	u32 width;               /* ceil(bytes_per_line/3) */
	size_t frame_bytes;      /* bytes_per_line * height */

	/* DRM client buffers */
	struct drm_client_dev client;
	struct drm_client_buffer *cbuf[DSI_STREAM_NUM_BUFS];
	struct iosys_map map[DSI_STREAM_NUM_BUFS];
	void *vaddr[DSI_STREAM_NUM_BUFS];
	u32 pitch[DSI_STREAM_NUM_BUFS];

	/* staging (user-mappable) */
	void *staging_base;
	size_t staging_total;
	void *staging[DSI_STREAM_NUM_BUFS];

	/* producer/consumer ring */
	atomic_t queued;
	int head;
	int tail;
	wait_queue_head_t wq_can_write;
	wait_queue_head_t wq_can_flip;
	struct task_struct *flip_thread;
	bool running;

	/* stats */
	u64 frames_in;
	u64 frames_flipped;

	/* misc chardev */
	struct miscdevice miscdev;
	bool misc_registered;

	/* work */
	struct work_struct start_work;
	bool want_start;

	atomic_t open_count;

	/* readiness for staging mmap */
	wait_queue_head_t wq_started;
	bool staging_ready;
};

/* Callback for when the last reference drops */
static void dsi_stream_dev_release_kref(struct kref *ref)
{
	struct dsi_stream_dev *s = container_of(ref, struct dsi_stream_dev, refcount);

	if (s->staging_base) {
		vfree(s->staging_base);
		s->staging_base = NULL;
	}

	kfree(s);
}

static inline struct dsi_stream_dev *to_dsi_stream(struct drm_panel *p)
{
	return container_of(p, struct dsi_stream_dev, panel);
}

static inline u32 dsi_stream_bpp_bits(const struct dsi_stream_dev *s)
{
	return s->fb_bpp * 8;
}

static inline u32 dsi_stream_htotal(const struct dsi_stream_dev *s)
{
	return s->width + 8 + 2 + 4;
}

static inline u32 dsi_stream_vtotal(const struct dsi_stream_dev *s)
{
	return s->height + 1 + 1 + 1;
}

static u32 dsi_stream_calc_mode_clock_khz(const struct dsi_stream_dev *s)
{
	u32 htotal = dsi_stream_htotal(s);
	u32 vtotal = dsi_stream_vtotal(s);

	if (s->target_dsi_clock_hz && s->lanes && s->fb_bpp) {
		u64 pclk_hz = (u64)s->target_dsi_clock_hz * 2ULL * s->lanes;

		/* DSI video payload: pixel_clock * bpp = dphy_clock * 2 * lanes. */
		pclk_hz = div_u64(pclk_hz + dsi_stream_bpp_bits(s) / 2,
					  dsi_stream_bpp_bits(s));
		return DIV_ROUND_CLOSEST_ULL(pclk_hz, 1000ULL);
	}

	return DIV_ROUND_CLOSEST_ULL((u64)htotal * vtotal * s->fps, 1000ULL);
}

static u64 dsi_stream_calc_dsi_clock_hz(const struct dsi_stream_dev *s,
						u32 mode_clock_khz)
{
	if (!s->lanes)
		return 0;

	/* D-PHY clock lane frequency; lane bit rate is 2x this value. */
	return div_u64((u64)mode_clock_khz * 1000ULL * dsi_stream_bpp_bits(s),
			     2ULL * s->lanes);
}

static void dsi_stream_update_clock_info(struct dsi_stream_dev *s)
{
	s->mode_clock_khz = dsi_stream_calc_mode_clock_khz(s);
	s->dsi_clock_hz = dsi_stream_calc_dsi_clock_hz(s, s->mode_clock_khz);
}

static void dsi_stream_zero_framebuffers(struct dsi_stream_dev *s)
{
	int i;

	for (i = 0; i < DSI_STREAM_NUM_BUFS; i++) {
		if (!s->vaddr[i])
			continue;

		if (s->map[i].is_iomem)
			memset_io(s->map[i].vaddr_iomem, 0,
				  (size_t)s->pitch[i] * s->height);
		else
			memset(s->map[i].vaddr, 0, (size_t)s->pitch[i] * s->height);
	}
}

/* ------------------- Copy helper: staging[idx] -> FB[idx] (no reordering) ------------------- */
static inline void dsi_stream_blit_to_fb(struct dsi_stream_dev *s, int idx)
{
	u32 y;
	for (y = 0; y < s->height; y++) {
		void *src = (u8 *)s->staging[idx] + (size_t)y * s->bytes_per_line;
		void *dst = (u8 *)s->vaddr[idx]    + (size_t)y * s->pitch[idx];
		memcpy(dst, src, s->bytes_per_line);
	}
}

/* Allocate user-mappable staging if not already allocated */
static int dsi_stream_alloc_staging(struct dsi_stream_dev *s)
{
	int i;
	if (s->staging_base)
		return 0;
	if (!s->frame_bytes)
		return -EINVAL;

	s->staging_total = PAGE_ALIGN((size_t)DSI_STREAM_NUM_BUFS * (size_t)s->frame_bytes);
	s->staging_base  = vmalloc_user(s->staging_total);
	if (!s->staging_base)
		return -ENOMEM;

	/* The idle payload is zero I/Q/RGB until userspace writes real samples. */
	memset(s->staging_base, 0, s->staging_total);

	for (i = 0; i < DSI_STREAM_NUM_BUFS; i++)
		s->staging[i] = (void *)((char *)s->staging_base + (size_t)i * s->frame_bytes);

	/* ready for mmap immediately */
	s->staging_ready = true;
	smp_wmb();
	wake_up_interruptible(&s->wq_started);
	return 0;
}

/* ------------------- DRM client + flip thread ------------------- */
static int dsi_stream_client_present(struct dsi_stream_dev *s, int idx)
{
	struct drm_mode_set *modeset;
	int ret, tries;

	if (!s->cbuf[idx])
		return -ENODEV;

	drm_client_for_each_modeset(modeset, &s->client) {
		bool has_dsi = false;
		unsigned int i;
		for (i = 0; i < modeset->num_connectors; i++) {
			struct drm_connector *conn = modeset->connectors[i];
			if (conn && conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
				has_dsi = true;
				break;
			}
		}
		if (!has_dsi)
			continue;
		modeset->fb = s->cbuf[idx]->fb;
		modeset->x = 0;
		modeset->y = 0;
	}

	for (tries = 0; tries < 8; tries++) {
		ret = drm_client_modeset_commit(&s->client);
		if (!ret)
			return 0;
		if (ret != -EBUSY)
			return ret;
		usleep_range(1000, 2000);
	}
	return -EBUSY;
}

static int dsi_stream_flip_thread(void *data)
{
	struct dsi_stream_dev *s = data;

	while (!kthread_should_stop()) {
		int ret;

		wait_event_interruptible(s->wq_can_flip,
			kthread_should_stop() || atomic_read(&s->queued) > 0);
		if (kthread_should_stop())
			break;
		if (atomic_read(&s->queued) == 0)
			continue;

		/* copy staged payload into FB, present */
		dsi_stream_blit_to_fb(s, s->tail);
		ret = dsi_stream_client_present(s, s->tail);
		if (ret == 0) {
			s->frames_flipped++;
			s->tail = (s->tail + 1) % DSI_STREAM_NUM_BUFS;
			atomic_dec(&s->queued);
			wake_up_interruptible(&s->wq_can_write);
		} else if (ret == -EBUSY) {
			usleep_range(1000, 2000);
		} else {
			drm_dbg(s->drm, "present failed: %d", ret);
			/* drop to keep forward progress */
			s->tail = (s->tail + 1) % DSI_STREAM_NUM_BUFS;
			atomic_dec(&s->queued);
			wake_up_interruptible(&s->wq_can_write);
		}
	}
	return 0;
}

static int dsi_stream_client_register(struct dsi_stream_dev *s)
{
	int ret, i;
	u32 fourcc = DRM_FORMAT_RGB888; /* wire format */

	if (!s->drm)
		return -EPROBE_DEFER;

	ret = drm_client_init(s->drm, &s->client, "dsi-stream", NULL);
	if (ret)
		return ret;
	drm_client_register(&s->client);

	ret = drm_client_modeset_probe(&s->client, s->width, s->height);
	if (ret)
		goto err_release;

	for (i = 0; i < DSI_STREAM_NUM_BUFS; i++) {
		struct drm_client_buffer *cb;

		cb = drm_client_framebuffer_create(&s->client, s->width, s->height, fourcc);
		if (IS_ERR(cb)) { ret = PTR_ERR(cb); goto err_release; }
		s->cbuf[i] = cb;

		ret = drm_client_buffer_vmap(cb, &s->map[i]);
		if (ret)
			goto err_release;
		s->vaddr[i] = s->map[i].vaddr;
		s->pitch[i] = s->cbuf[i]->pitch;
	}

	if (s->zero_idle_on_start) {
		dsi_stream_zero_framebuffers(s);
		ret = dsi_stream_client_present(s, 0);
		if (ret)
			dev_warn(s->dev, "initial zero-frame present failed: %d\n", ret);
	}

	/* staging already allocated at probe */
	atomic_set(&s->queued, 0);
	s->head = s->tail = 0;
	wake_up_interruptible(&s->wq_can_write);

	s->flip_thread = kthread_run(dsi_stream_flip_thread, s, "dsi-stream-flip");
	if (IS_ERR(s->flip_thread)) {
		ret = PTR_ERR(s->flip_thread);
		s->flip_thread = NULL;
		goto err_release;
	}
	return 0;

err_release:
	if (s->flip_thread && !IS_ERR(s->flip_thread)) {
		kthread_stop(s->flip_thread);
		s->flip_thread = NULL;
	}
	for (i = 0; i < DSI_STREAM_NUM_BUFS; i++) {
		if (s->cbuf[i]) {
			drm_client_buffer_vunmap(s->cbuf[i]);
			drm_client_framebuffer_delete(s->cbuf[i]);
			s->cbuf[i] = NULL;
		}
	}
	/* Set client.dev to NULL after releasing to avoid double-free paging error */
	if (s->client.dev) {
		drm_client_release(&s->client);
		s->client.dev = NULL; 
	}
	return ret;
}

static int dsi_stream_client_unregister(struct dsi_stream_dev *s)
{
	int i;
	if (s->flip_thread) {
		kthread_stop(s->flip_thread);
		s->flip_thread = NULL;
	}
	s->running = false;
	wake_up_interruptible(&s->wq_can_write);
	wake_up_interruptible(&s->wq_can_flip);

	/* Keep staging mapping valid across enable/disable cycles */
	for (i = 0; i < DSI_STREAM_NUM_BUFS; i++) {
		if (s->cbuf[i]) {
			drm_client_buffer_vunmap(s->cbuf[i]);
			drm_client_framebuffer_delete(s->cbuf[i]);
			s->cbuf[i] = NULL;
		}
	}

	if (s->client.dev){
		drm_client_release(&s->client);
		s->client.dev = NULL;
	}
	return 0;
}

/* ------------------- char device ops ------------------- */
static int dsi_stream_open(struct inode *ino, struct file *f)
{
	struct miscdevice *m = f->private_data;
	struct dsi_stream_dev *s = dev_get_drvdata(m->this_device);
	
	kref_get(&s->refcount);

	f->private_data = s;
	f->f_pos = 0;
	if (atomic_inc_return(&s->open_count) != 1) {
		atomic_dec(&s->open_count);
		kref_put(&s->refcount, dsi_stream_dev_release_kref);
		return -EBUSY;
	}
	s->want_start = true;
	schedule_work(&s->start_work);
	return 0;
}

static ssize_t dsi_stream_write(struct file *f, const char __user *ubuf,
                                size_t len, loff_t *ppos)
{
    struct dsi_stream_dev *s = f->private_data;
    size_t remaining = len;

    if (!s->running || !s->staging_base || s->frame_bytes == 0)
        return -EAGAIN;

    while (remaining) {
        size_t off_in_frame = (*ppos); 
        
        if (off_in_frame >= s->frame_bytes) {
             off_in_frame = 0;
             *ppos = 0;
        }

        size_t can_take     = s->frame_bytes - off_in_frame;
        size_t chunk        = min(remaining, can_take);
        void *dst           = (u8 *)s->staging[s->head] + off_in_frame;
        int ret;

        if (off_in_frame == 0) {
            ret = wait_event_interruptible(
                s->wq_can_write,
                !s->running || (atomic_read(&s->queued) < (DSI_STREAM_NUM_BUFS - 1)));
            if (ret)
                return (len == remaining) ? ret : (ssize_t)(len - remaining);
            if (!s->running)
                return (ssize_t)(len - remaining ? : -EPIPE);
        }

        if (copy_from_user(dst, ubuf, chunk))
            return (len == remaining) ? -EFAULT : (ssize_t)(len - remaining);

        ubuf       += chunk;
        remaining  -= chunk;
        *ppos      += chunk;

        if (*ppos >= s->frame_bytes) {
            s->frames_in++;
            s->head = (s->head + 1) % DSI_STREAM_NUM_BUFS;
            atomic_inc(&s->queued);
            wake_up_interruptible(&s->wq_can_flip);
            
            *ppos = 0;
        }
    }
    return (ssize_t)len;
}

static int dsi_stream_release(struct inode *ino, struct file *f)
{
	struct dsi_stream_dev *s = f->private_data;
	atomic_dec_if_positive(&s->open_count);

	kref_put(&s->refcount, dsi_stream_dev_release_kref);
	return 0;
}

static __poll_t dsi_stream_poll(struct file *f, poll_table *wait){
	struct dsi_stream_dev *s = f->private_data;
	__poll_t mask = 0;
	poll_wait(f, &s->wq_can_write, wait);
	if (s->running && atomic_read(&s->queued) < DSI_STREAM_NUM_BUFS - 1)
		mask |= POLLOUT | POLLWRNORM;
	return mask;
}

static long dsi_stream_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct dsi_stream_dev *s = f->private_data;

    switch (cmd) {
    case DSI_IOC_GET_FB_INFO: {
        struct dsi_fb_info info = {
            .fb_bytes = s->frame_bytes,
            .fb_count = DSI_STREAM_NUM_BUFS,
            .head     = s->head,
            .tail     = s->tail,
            .queued   = atomic_read(&s->queued),
        };
        if (copy_to_user((void __user *)arg, &info, sizeof(info)))
            return -EFAULT;
        return 0;
    }
    case DSI_IOC_QUEUE_NEXT:
        break;
    default:
        return -ENOTTY;
    }

    if (!s->running)
        return -EAGAIN;

    if (atomic_read(&s->queued) >= (DSI_STREAM_NUM_BUFS - 1))
        return -EAGAIN;

    smp_wmb();

    s->frames_in++;
    s->head = (s->head + 1) % DSI_STREAM_NUM_BUFS;
    atomic_inc(&s->queued);
    wake_up_interruptible(&s->wq_can_flip);
    return 0;
}

static int dsi_stream_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct dsi_stream_dev *s = f->private_data;
	const size_t total  = (size_t)DSI_STREAM_NUM_BUFS * s->frame_bytes;
	const size_t want   = vma->vm_end - vma->vm_start;
	const size_t palign = PAGE_ALIGN(total);
	long retw;

	retw = wait_event_interruptible_timeout(s->wq_started,
		s->staging_ready, msecs_to_jiffies(1000));
	if (retw < 0)
		return retw; 
	if (!s->staging_ready)
		return -EAGAIN; 

	if (want != total && want != palign)
		return -EINVAL;
	if (!s->staging_base || s->staging_total < palign)
		return -ENXIO;

	vma->vm_pgoff = 0;
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	return remap_vmalloc_range(vma, s->staging_base, 0);
}

static const struct file_operations dsi_stream_fops = {
	.owner          = THIS_MODULE,
	.open           = dsi_stream_open,
	.release        = dsi_stream_release,
	.poll           = dsi_stream_poll,
	.unlocked_ioctl = dsi_stream_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = dsi_stream_ioctl,
#endif
	.mmap           = dsi_stream_mmap,
	.write          = dsi_stream_write,
	.llseek         = no_llseek,
};

/* ------------------- panel funcs ------------------- */
static int dsi_stream_panel_get_modes(struct drm_panel *panel, struct drm_connector *conn)
{
	struct dsi_stream_dev *s = to_dsi_stream(panel);
	struct drm_display_mode *m;

	s->drm = conn->dev;
	m = drm_mode_create(conn->dev);
	if (!m)
		return 0;

	m->hdisplay    = s->width;
	m->hsync_start = m->hdisplay + 8;
	m->hsync_end   = m->hsync_start + 2;
	m->htotal      = dsi_stream_htotal(s);
	m->vdisplay    = s->height;
	m->vsync_start = m->vdisplay + 1;
	m->vsync_end   = m->vsync_start + 1;
	m->vtotal      = dsi_stream_vtotal(s);

	dsi_stream_update_clock_info(s);
	m->clock = s->mode_clock_khz;
	m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	strscpy(m->name, "DSI-STREAM", sizeof(m->name));
	drm_mode_set_name(m);
	drm_mode_probed_add(conn, m);
	conn->display_info.width_mm = 0;
	conn->display_info.height_mm = 0;

	if (s->want_start)
		schedule_work(&s->start_work);
	return 1;
}

static int dsi_stream_panel_prepare(struct drm_panel *panel)
{
	struct dsi_stream_dev *s = to_dsi_stream(panel);
	if (s->prepared) return 0;
	s->prepared = true;
	return 0;
}

static int dsi_stream_panel_enable(struct drm_panel *panel)
{
	struct dsi_stream_dev *s = to_dsi_stream(panel);
	if (s->enabled) return 0;
	s->enabled = true;
	schedule_work(&s->start_work);
	dev_info(s->dev, "panel enabled; start_work queued\n");
	return 0;
}

static int dsi_stream_panel_disable(struct drm_panel *panel)
{
	struct dsi_stream_dev *s = to_dsi_stream(panel);
	if (!s->enabled) return 0;
	s->running = false;
	wake_up_interruptible(&s->wq_can_write);
	wake_up_interruptible(&s->wq_can_flip);

	dsi_stream_client_unregister(s);
	s->enabled = false;
	dev_info(s->dev, "panel disabled; flip thread stopped\n");
	return 0;
}

static int dsi_stream_panel_unprepare(struct drm_panel *panel)
{
	struct dsi_stream_dev *s = to_dsi_stream(panel);
	s->prepared = false;
	return 0;
}

static const struct drm_panel_funcs dsi_stream_panel_funcs = {
	.prepare   = dsi_stream_panel_prepare,
	.enable    = dsi_stream_panel_enable,
	.disable   = dsi_stream_panel_disable,
	.unprepare = dsi_stream_panel_unprepare,
	.get_modes = dsi_stream_panel_get_modes,
};

/* ------------------- DT + probe/remove ------------------- */
static int dsi_stream_parse_dt(struct dsi_stream_dev *s)
{
	struct device_node *np = s->dev->of_node;

	/* defaults */
	s->bytes_per_line = 3072;
	s->height = 1080;
	s->fps = 26;
	s->lanes = 2;
	s->continuous_clk = true;
	s->fb_bpp = 3;
	s->target_dsi_clock_hz = 0;
	s->start_on_load = true;
	s->zero_idle_on_start = true;

	of_property_read_u32(np, "acme,bytes-per-line", &s->bytes_per_line);
	of_property_read_u32(np, "acme,height", &s->height);
	of_property_read_u32(np, "acme,fps", &s->fps);
	of_property_read_u32(np, "acme,lanes", &s->lanes);
	of_property_read_u32(np, "acme,dsi-clock-hz", &s->target_dsi_clock_hz);
	if (of_property_read_bool(np, "acme,continuous-clock"))
		s->continuous_clk = true;
	if (of_property_read_bool(np, "acme,non-continuous-clock"))
		s->continuous_clk = false;
	if (of_property_read_bool(np, "acme,start-on-load"))
		s->start_on_load = true;
	if (of_property_read_bool(np, "acme,no-start-on-load"))
		s->start_on_load = false;
	if (of_property_read_bool(np, "acme,zero-idle-on-start"))
		s->zero_idle_on_start = true;
	if (of_property_read_bool(np, "acme,no-zero-idle"))
		s->zero_idle_on_start = false;

	if (!s->lanes || !s->fb_bpp || !s->height || !s->bytes_per_line)
		return -EINVAL;

	s->width = DIV_ROUND_UP(s->bytes_per_line, 3);
	s->frame_bytes = (size_t)s->bytes_per_line * s->height;
	dsi_stream_update_clock_info(s);
	return 0;
}

static void dsi_stream_start_work(struct work_struct *work);

static int dsi_stream_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct dsi_stream_dev *s;
	int ret;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) return -ENOMEM;
	
	kref_init(&s->refcount);

	s->dev = dev;
	s->dsi = dsi;
	INIT_WORK(&s->start_work, dsi_stream_start_work);
	atomic_set(&s->open_count, 0);
	init_waitqueue_head(&s->wq_started);
	init_waitqueue_head(&s->wq_can_write);
   	init_waitqueue_head(&s->wq_can_flip);

	s->staging_ready = false;

	ret = dsi_stream_parse_dt(s);
	if (ret) goto err_put;

	ret = dsi_stream_alloc_staging(s);
	if (ret) {
		dev_err(dev, "staging alloc failed: %d\n", ret);
		goto err_put;
	}

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
	                  MIPI_DSI_MODE_VIDEO_BURST |
	                  MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
	                  MIPI_DSI_MODE_VIDEO_HSE;
	if (!s->continuous_clk)
		dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes  = s->lanes;

	if (s->start_on_load)
		s->want_start = true;

	drm_panel_init(&s->panel, dev, &dsi_stream_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&s->panel);
	mipi_dsi_set_drvdata(dsi, s);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&s->panel);
		goto err_put;
	}

	s->miscdev.minor  = MISC_DYNAMIC_MINOR;
	s->miscdev.name   = "dsi_stream0";
	s->miscdev.fops   = &dsi_stream_fops;
	s->miscdev.parent = dev;
	s->miscdev.mode   = 0666;
	ret = misc_register(&s->miscdev);
	if (ret) {
		mipi_dsi_detach(dsi);
		drm_panel_remove(&s->panel);
		goto err_put;
	}
	dev_set_drvdata(s->miscdev.this_device, s);
	s->misc_registered = true;
	device_create_file(s->miscdev.this_device, &dev_attr_frames_in);
	device_create_file(s->miscdev.this_device, &dev_attr_frames_flipped);
	device_create_file(s->miscdev.this_device, &dev_attr_queued);
	device_create_file(s->miscdev.this_device, &dev_attr_ring);
	device_create_file(s->miscdev.this_device, &dev_attr_dsi_clock_hz);
	device_create_file(s->miscdev.this_device, &dev_attr_pixel_clock_khz);

	if (s->want_start)
		schedule_work(&s->start_work);

	dev_info(dev, "dsi-stream ready (raw bytes): %u B/line, %u lines, %u fps, %u lanes, pixel clock %u kHz, DSI clock %llu Hz, start_on_load=%d, zero_idle=%d\n",
	         s->bytes_per_line, s->height, s->fps, s->lanes,
	         s->mode_clock_khz, s->dsi_clock_hz,
	         s->start_on_load, s->zero_idle_on_start);
	return 0;

err_put:
	kref_put(&s->refcount, dsi_stream_dev_release_kref);
	return ret;
}

static void dsi_stream_remove(struct mipi_dsi_device *dsi)
{
	struct dsi_stream_dev *s = mipi_dsi_get_drvdata(dsi);
	if (!s) return;
	cancel_work_sync(&s->start_work);
	if (s->misc_registered) {
		device_remove_file(s->miscdev.this_device, &dev_attr_pixel_clock_khz);
		device_remove_file(s->miscdev.this_device, &dev_attr_dsi_clock_hz);
		device_remove_file(s->miscdev.this_device, &dev_attr_ring);
		device_remove_file(s->miscdev.this_device, &dev_attr_queued);
		device_remove_file(s->miscdev.this_device, &dev_attr_frames_flipped);
		device_remove_file(s->miscdev.this_device, &dev_attr_frames_in);
		misc_deregister(&s->miscdev);
		s->misc_registered = false;
	}
	dsi_stream_client_unregister(s);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&s->panel);

	kref_put(&s->refcount, dsi_stream_dev_release_kref);
}

static void dsi_stream_start_work(struct work_struct *work)
{
	struct dsi_stream_dev *s = container_of(work, struct dsi_stream_dev, start_work);
	int ret;

	if (s->flip_thread || s->client.dev)
		return;
	if (!s->drm)
		return;

	ret = dsi_stream_client_register(s);
	if (ret) {
		dev_warn(s->dev, "client_register failed: %d\n", ret);
		return;
	}
	s->running = true;

	dev_info(s->dev, "client started; zero idle frame active; DSI clock %llu Hz (lane bit rate %llu Hz)\n",
		 s->dsi_clock_hz, s->dsi_clock_hz * 2ULL);
}

/* Sysfs */
static ssize_t frames_in_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	return sysfs_emit(buf, "%llu\n", s->frames_in);
}
static ssize_t frames_flipped_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	return sysfs_emit(buf, "%llu\n", s->frames_flipped);
}
static ssize_t queued_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	return sysfs_emit(buf, "%d\n", atomic_read(&s->queued));
}
static ssize_t ring_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	return sysfs_emit(buf, "%d %d\n", s->head, s->tail);
}
static ssize_t dsi_clock_hz_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	dsi_stream_update_clock_info(s);
	return sysfs_emit(buf, "%llu\n", s->dsi_clock_hz);
}
static ssize_t pixel_clock_khz_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct dsi_stream_dev *s = dev_get_drvdata(d);
	dsi_stream_update_clock_info(s);
	return sysfs_emit(buf, "%u\n", s->mode_clock_khz);
}

static const struct of_device_id dsi_stream_of_match[] = {
	{ .compatible = "acme,dsi-stream-panel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dsi_stream_of_match);

static struct mipi_dsi_driver dsi_stream_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = dsi_stream_of_match,
	},
	.probe  = dsi_stream_probe,
	.remove = dsi_stream_remove,
};

module_mipi_dsi_driver(dsi_stream_driver);

MODULE_DESCRIPTION("DSI byte-stream panel (/dev/dsi_stream0): staging-mmap, RGB888, zero-idle autostart");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("open.space");
