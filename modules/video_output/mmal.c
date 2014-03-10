/*****************************************************************************
 * mmal.c: MMAL-based vout plugin for Raspberry Pi
 *****************************************************************************
 * Copyright © 2014 jusst technologies GmbH
 * $Id$
 *
 * Authors: Dennis Hamester <dennis.hamester@gmail.com>
 *          Julian Scheel <jscheel@jusst.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_threads.h>
#include <vlc_vout_display.h>

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/vmcs_host/vc_tvservice.h>
#include <interface/mmal/mmal_logging.h>

#define MMAL_OPAQUE_MAGIC VLC_FOURCC('M', 'M', 'A', 'L')
#define MAX_BUFFERS_IN_TRANSIT 2

static int Open(vlc_object_t *);
static void Close(vlc_object_t *);

vlc_module_begin()
    set_shortname(N_("MMAL vout"))
    set_description(N_("MMAL-based vout plugin for Raspberry Pi"))
    set_capability("vout display", 0)
    add_shortcut("mmal_vout")
    set_callbacks(Open, Close)
vlc_module_end()

struct vout_display_sys_t {
    bool opaque;
    MMAL_COMPONENT_T *component;
    MMAL_PORT_T *input;
    MMAL_POOL_T *pool;
    unsigned num_buffers;
    uint32_t buffer_size;
    picture_t **pictures;
    picture_pool_t *picture_pool;
    plane_t planes[3];
    vlc_mutex_t buffer_mutex;
    vlc_cond_t buffer_cond;
    int buffers_in_transit;
    unsigned display_width;
    unsigned display_height;
    vlc_mutex_t manage_mutex;
    bool need_configure_display;
};

struct picture_sys_t {
    vout_display_t *vd;
    MMAL_BUFFER_HEADER_T *buffer;
    bool displayed;
};

/* Utility functions */
static uint32_t align(uint32_t x, uint32_t y);
static int configure_display(vout_display_t *vd, const vout_display_cfg_t *cfg, const video_format_t *fmt);

/* VLC vout display callbacks */
static picture_pool_t *vd_pool(vout_display_t *vd, unsigned count);
/*static void vd_prepare(vout_display_t *vd, picture_t *pic, subpicture_t *sub_pic);*/
static void vd_display(vout_display_t *vd, picture_t *picture, subpicture_t *sub_picture);
static int vd_control(vout_display_t *vd, int query, va_list args);
static void vd_manage(vout_display_t *vd);

/* VLC picture pool */
static int picture_lock(picture_t *picture);
static void picture_unlock(picture_t *picture);

/* MMAL callbacks */
static void control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void* pool_allocator_alloc(void *context, uint32_t size);
static void pool_allocator_free(void *context, void *mem);

/* TV service */
static int query_resolution(vout_display_t *vd, unsigned *width, unsigned *height);
static void tvservice_cb(void *callback_data, uint32_t reason, uint32_t param1, uint32_t param2);

static int Open(vlc_object_t *object)
{
    vout_display_t *vd = (vout_display_t *)object;
    vout_display_sys_t *sys;
    uint32_t buffer_pitch, buffer_height;
    vout_display_place_t place;
    MMAL_DISPLAYREGION_T display_region;
    uint32_t offsets[3];
    MMAL_STATUS_T status;
    int ret = VLC_SUCCESS;
    unsigned i;

    sys = calloc(1, sizeof(struct vout_display_sys_t));
    if (!sys) {
        msg_Err(vd, "Failed to allocate struct vout_display_sys_t");
        ret = VLC_ENOMEM;
        goto out;
    }
    vd->sys = sys;

    bcm_host_init();

    vd->info.has_hide_mouse = true;
    sys->opaque = vd->fmt.i_chroma == MMAL_OPAQUE_MAGIC;

    if (!sys->opaque)
        vd->fmt.i_chroma = VLC_CODEC_I420;
    vd->fmt.i_sar_num = vd->source.i_sar_num;
    vd->fmt.i_sar_den = vd->source.i_sar_den;

    buffer_pitch = align(vd->fmt.i_width, 32);
    buffer_height = align(vd->fmt.i_height, 16);
    sys->buffer_size = 3 * buffer_pitch * buffer_height / 2;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &sys->component);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to create MMAL component %s (status=%"PRIx32" %s)", MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->component->control->userdata = (struct MMAL_PORT_USERDATA_T *)vd;
    status = mmal_port_enable(sys->component->control, control_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to enable control port %s (status=%"PRIx32" %s)", sys->component->control->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->input = sys->component->input[0];
    sys->input->userdata = (struct MMAL_PORT_USERDATA_T *)vd;
    if (sys->opaque)
        sys->input->format->encoding = MMAL_ENCODING_OPAQUE;
    else
        sys->input->format->encoding = MMAL_ENCODING_I420;
    sys->input->format->es->video.width = vd->fmt.i_width;
    sys->input->format->es->video.height = vd->fmt.i_height;
    sys->input->format->es->video.crop.x = 0;
    sys->input->format->es->video.crop.y = 0;
    sys->input->format->es->video.crop.width = vd->fmt.i_width;
    sys->input->format->es->video.crop.height = vd->fmt.i_height;
    sys->input->format->es->video.par.num = vd->source.i_sar_num;
    sys->input->format->es->video.par.den = vd->source.i_sar_den;

    status = mmal_port_format_commit(sys->input);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to commit format for input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }
    sys->input->buffer_size = sys->input->buffer_size_recommended;

    vout_display_PlacePicture(&place, &vd->source, vd->cfg, false);
    display_region.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
    display_region.hdr.size = sizeof(MMAL_DISPLAYREGION_T);
    display_region.fullscreen = MMAL_FALSE;
    display_region.src_rect.x = vd->fmt.i_x_offset;
    display_region.src_rect.y = vd->fmt.i_y_offset;
    display_region.src_rect.width = vd->fmt.i_visible_width;
    display_region.src_rect.height = vd->fmt.i_visible_height;
    display_region.dest_rect.x = place.x;
    display_region.dest_rect.y = place.y;
    display_region.dest_rect.width = place.width;
    display_region.dest_rect.height = place.height;
    display_region.set = MMAL_DISPLAY_SET_FULLSCREEN | MMAL_DISPLAY_SET_SRC_RECT | MMAL_DISPLAY_SET_DEST_RECT;
    status = mmal_port_parameter_set(sys->input, &display_region.hdr);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to set display region (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    offsets[0] = 0;
    for (i = 0; i < 3; ++i) {
        sys->planes[i].i_lines = buffer_height;
        sys->planes[i].i_pitch = buffer_pitch;
        sys->planes[i].i_visible_lines = vd->fmt.i_visible_height;
        sys->planes[i].i_visible_pitch = vd->fmt.i_visible_width;

        if (i > 0) {
            offsets[i] = offsets[i - 1] + sys->planes[i - 1].i_pitch * sys->planes[i - 1].i_lines;
            sys->planes[i].i_lines /= 2;
            sys->planes[i].i_pitch /= 2;
            sys->planes[i].i_visible_lines /= 2;
            sys->planes[i].i_visible_pitch /= 2;
        }

        sys->planes[i].p_pixels = (uint8_t *)offsets[i];
    }

    vlc_mutex_init(&sys->buffer_mutex);
    vlc_cond_init(&sys->buffer_cond);
    vlc_mutex_init(&sys->manage_mutex);

    vd->pool = vd_pool;
    /*vd->prepare = vd_prepare;*/
    vd->display = vd_display;
    vd->control = vd_control;
    vd->manage = vd_manage;

    vc_tv_register_callback(tvservice_cb, vd);

    if (query_resolution(vd, &sys->display_width, &sys->display_height) >= 0) {
        vout_display_SendEventDisplaySize(vd, sys->display_width, sys->display_height, vd->cfg->is_fullscreen);
    }
    else {
        sys->display_width = vd->cfg->display.width;
        sys->display_height = vd->cfg->display.height;
    }

out:
    if (ret != VLC_SUCCESS)
        Close(object);

    return ret;
}

static void Close(vlc_object_t *object)
{
    vout_display_t *vd = (vout_display_t *)object;
    vout_display_sys_t *sys = vd->sys;
    mtime_t timeout;
    unsigned i;

    if (!sys)
        return;

    vc_tv_unregister_callback(tvservice_cb);

    if (sys->component) {
        if (sys->component->is_enabled)
            mmal_component_disable(sys->component);

        if (sys->component->control->is_enabled)
            mmal_port_disable(sys->component->control);
    }

    if (sys->input && sys->input->is_enabled)
        mmal_port_disable(sys->input);

    if (sys->pool) {
        if (mmal_queue_length(sys->pool->queue) != sys->num_buffers) {
            msg_Dbg(vd, "Waiting for %d buffers to be released", sys->num_buffers - mmal_queue_length(sys->pool->queue));

            timeout = mdate() + 1000000;
            vlc_mutex_lock(&sys->buffer_mutex);
            while (mmal_queue_length(sys->pool->queue) != sys->num_buffers)
                if (vlc_cond_timedwait(&sys->buffer_cond, &sys->buffer_mutex, timeout) != 0)
                    break;
            vlc_mutex_unlock(&sys->buffer_mutex);

            if (mmal_queue_length(sys->pool->queue) != sys->num_buffers)
                msg_Err(vd, "Timeout while waiting for buffers to be released");
        }

        mmal_pool_destroy(sys->pool);
    }

    if (sys->component)
        mmal_component_release(sys->component);

    if (sys->picture_pool)
        picture_pool_Delete(sys->picture_pool);
    else
        for (i = 0; i < sys->num_buffers; ++i)
            if (sys->pictures[i])
                picture_Release(sys->pictures[i]);

    vlc_mutex_destroy(&sys->buffer_mutex);
    vlc_cond_destroy(&sys->buffer_cond);
    vlc_mutex_destroy(&sys->manage_mutex);

    free(sys);

    bcm_host_deinit();
}

static uint32_t align(uint32_t x, uint32_t y) {
    uint32_t mod = x % y;
    if (mod == 0)
        return x;
    else
        return x + y - mod;
}

static int configure_display(vout_display_t *vd, const vout_display_cfg_t *cfg, const video_format_t *fmt)
{
    vout_display_sys_t *sys = vd->sys;
    vout_display_place_t place;
    MMAL_DISPLAYREGION_T display_region;
    MMAL_STATUS_T status;

    if (!cfg && !fmt)
        return -EINVAL;

    if (fmt) {
        sys->input->format->es->video.par.num = fmt->i_sar_num;
        sys->input->format->es->video.par.den = fmt->i_sar_den;

        status = mmal_port_format_commit(sys->input);
        if (status != MMAL_SUCCESS) {
            msg_Err(vd, "Failed to commit format for input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
            return -EINVAL;
        }
    }
    else {
        fmt = &vd->fmt;
    }

    if (!cfg)
        cfg = vd->cfg;

    vout_display_PlacePicture(&place, fmt, cfg, false);

    display_region.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
    display_region.hdr.size = sizeof(MMAL_DISPLAYREGION_T);
    display_region.fullscreen = MMAL_FALSE;
    display_region.src_rect.x = fmt->i_x_offset;
    display_region.src_rect.y = fmt->i_y_offset;
    display_region.src_rect.width = fmt->i_visible_width;
    display_region.src_rect.height = fmt->i_visible_height;
    display_region.dest_rect.x = place.x;
    display_region.dest_rect.y = place.y;
    display_region.dest_rect.width = place.width;
    display_region.dest_rect.height = place.height;
    display_region.set = MMAL_DISPLAY_SET_FULLSCREEN | MMAL_DISPLAY_SET_SRC_RECT | MMAL_DISPLAY_SET_DEST_RECT;
    status = mmal_port_parameter_set(sys->input, &display_region.hdr);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to set display region (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        return -EINVAL;
    }

    return 0;
}

static picture_pool_t *vd_pool(vout_display_t *vd, unsigned count)
{
    vout_display_sys_t *sys = vd->sys;
    picture_resource_t picture_res;
    picture_pool_configuration_t picture_pool_cfg;
    video_format_t fmt = vd->fmt;
    MMAL_STATUS_T status;
    unsigned i;

    if (sys->picture_pool) {
        if (sys->num_buffers < count)
            msg_Warn(vd, "Picture pool with %u pictures requested, but we already have one with %u pictures", count, sys->num_buffers);

        goto out;
    }

    if (count < sys->input->buffer_num_recommended)
        count = sys->input->buffer_num_recommended;

    msg_Dbg(vd, "Creating picture pool with %u pictures", count);

    sys->input->buffer_num = count;
    status = mmal_port_enable(sys->input, input_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to enable input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
        goto out;
    }

    status = mmal_component_enable(sys->component);
    if (status != MMAL_SUCCESS) {
        msg_Err(vd, "Failed to enable component %s (status=%"PRIx32" %s)", sys->component->name, status, mmal_status_to_string(status));
        goto out;
    }

    sys->pool = mmal_pool_create_with_allocator(count, sys->input->buffer_size, vd, pool_allocator_alloc, pool_allocator_free);
    if (!sys->pool) {
        msg_Err(vd, "Failed to create MMAL pool for %u buffers of size %"PRIu32, count, sys->input->buffer_size);
        goto out;
    }
    sys->num_buffers = count;

    if (sys->opaque)
        fmt.i_chroma = VLC_CODEC_I420;

    memset(&picture_res, 0, sizeof(picture_resource_t));
    sys->pictures = calloc(sys->num_buffers, sizeof(picture_t *));
    for (i = 0; i < sys->num_buffers; ++i) {
        picture_res.p_sys = malloc(sizeof(picture_sys_t));
        picture_res.p_sys->vd = vd;
        picture_res.p_sys->buffer = NULL;

        sys->pictures[i] = picture_NewFromResource(&fmt, &picture_res);
        if (!sys->pictures[i]) {
            msg_Err(vd, "Failed to create picture");
            free(picture_res.p_sys);
            goto out;
        }

        if(sys->opaque)
            sys->pictures[i]->format.i_chroma = MMAL_OPAQUE_MAGIC;
    }

    memset(&picture_pool_cfg, 0, sizeof(picture_pool_configuration_t));
    picture_pool_cfg.picture_count = sys->num_buffers;
    picture_pool_cfg.picture = sys->pictures;
    picture_pool_cfg.lock = picture_lock;
    picture_pool_cfg.unlock = picture_unlock;

    sys->picture_pool = picture_pool_NewExtended(&picture_pool_cfg);
    if (!sys->picture_pool) {
        msg_Err(vd, "Failed to create picture pool");
        goto out;
    }

out:
    return sys->picture_pool;
}

/*static void vd_prepare(vout_display_t *vd, picture_t *pic, subpicture_t *sub_pic)*/
/*{*/
/*}*/

static void vd_display(vout_display_t *vd, picture_t *picture, subpicture_t *sub_picture)
{
    vout_display_sys_t *sys = vd->sys;
    picture_sys_t *pic_sys = picture->p_sys;
    MMAL_BUFFER_HEADER_T *buffer = pic_sys->buffer;
    MMAL_STATUS_T status;

    if (!pic_sys->displayed || !sys->opaque) {
        buffer->cmd = 0;
        buffer->length = sys->input->buffer_size;

        vlc_mutex_lock(&sys->buffer_mutex);
        while (sys->buffers_in_transit >= MAX_BUFFERS_IN_TRANSIT)
            vlc_cond_wait(&sys->buffer_cond, &sys->buffer_mutex);

        int ii;
        printf("vout: ");
        for (ii = 0; ii < 16; ++ii)
            printf("%"PRIx32" ", ((uint32_t*)buffer->data)[ii]);
        printf("\n");

        status = mmal_port_send_buffer(sys->input, buffer);
        if (status == MMAL_SUCCESS) {
            ++sys->buffers_in_transit;
        }
        else {
            msg_Err(vd, "Failed to send buffer to input port. Frame dropped");
            picture_Release(picture);
        }
        vlc_mutex_unlock(&sys->buffer_mutex);

        pic_sys->displayed = true;
    }
    else {
        picture_Release(picture);
    }

    if (sub_picture)
        subpicture_Delete(sub_picture);
}

static int vd_control(vout_display_t *vd, int query, va_list args)
{
    vout_display_sys_t *sys = vd->sys;
    vout_display_cfg_t cfg;
    const vout_display_cfg_t *tmp_cfg;
    video_format_t fmt;
    const video_format_t *tmp_fmt;
    int ret = VLC_EGENERIC;

    switch (query) {
        case VOUT_DISPLAY_HIDE_MOUSE:
        case VOUT_DISPLAY_CHANGE_WINDOW_STATE:
            ret = VLC_SUCCESS;
            break;

        case VOUT_DISPLAY_CHANGE_FULLSCREEN:
            tmp_cfg = va_arg(args, const vout_display_cfg_t *);
            vout_display_SendEventDisplaySize(vd, sys->display_width, sys->display_height, tmp_cfg->is_fullscreen);
            ret = VLC_SUCCESS;
            break;

        case VOUT_DISPLAY_CHANGE_DISPLAY_SIZE:
            tmp_cfg = va_arg(args, const vout_display_cfg_t *);
            if (tmp_cfg->display.width == sys->display_width && tmp_cfg->display.height == sys->display_height) {
                cfg = *vd->cfg;
                cfg.display.width = sys->display_width;
                cfg.display.height = sys->display_height;
                if (configure_display(vd, &cfg, NULL) >= 0)
                    ret = VLC_SUCCESS;
            }
            break;

        case VOUT_DISPLAY_CHANGE_DISPLAY_FILLED:
            cfg = *vd->cfg;
            tmp_cfg = va_arg(args, const vout_display_cfg_t *);
            cfg.is_display_filled = tmp_cfg->is_display_filled;
            if (configure_display(vd, &cfg, NULL) >= 0)
                ret = VLC_SUCCESS;
            break;

        /* TODO: Figure out how to implement zoom. Setting src_rect might do
         * job after cropping is fixed.
         * case VOUT_DISPLAY_CHANGE_ZOOM:
         *  cfg = *vd->cfg;
         *  tmp_cfg = va_arg(args, const vout_display_cfg_t *);
         *  cfg.zoom = tmp_cfg->zoom;
         *  if (configure_display(vd, &cfg, NULL) >= 0)
         *      ret = VLC_SUCCESS;
         *  break;
         */

        case VOUT_DISPLAY_CHANGE_SOURCE_ASPECT:
            fmt = vd->fmt;
            tmp_fmt = va_arg(args, const video_format_t *);
            fmt.i_sar_num = tmp_fmt->i_sar_num;
            fmt.i_sar_den = tmp_fmt->i_sar_den;
            if (configure_display(vd, NULL, &fmt) >= 0)
                ret = VLC_SUCCESS;
            break;

        case VOUT_DISPLAY_CHANGE_SOURCE_CROP:
            fmt = vd->fmt;
            tmp_fmt = va_arg(args, const video_format_t *);
            fmt.i_x_offset = tmp_fmt->i_x_offset;
            fmt.i_y_offset = tmp_fmt->i_y_offset;
            fmt.i_visible_width = tmp_fmt->i_visible_width;
            fmt.i_visible_height = tmp_fmt->i_visible_height;
            if (configure_display(vd, NULL, &fmt) >= 0)
                ret = VLC_SUCCESS;
            break;

        case VOUT_DISPLAY_CHANGE_ZOOM:
        case VOUT_DISPLAY_RESET_PICTURES:
        case VOUT_DISPLAY_GET_OPENGL:
            msg_Warn(vd, "Unsupported control query %d", query);
            break;

        default:
            msg_Warn(vd, "Unknown control query %d", query);
            break;
    }

    return ret;
}

static void vd_manage(vout_display_t *vd)
{
    vout_display_sys_t *sys = vd->sys;
    unsigned width, height;

    vlc_mutex_lock(&sys->manage_mutex);

    if (sys->need_configure_display) {
        if (query_resolution(vd, &width, &height) >= 0) {
            sys->display_width = width;
            sys->display_height = height;
            vout_display_SendEventDisplaySize(vd, width, height, vd->cfg->is_fullscreen);
        }

        sys->need_configure_display = false;
    }

    vlc_mutex_unlock(&sys->manage_mutex);
}

static int picture_lock(picture_t *picture)
{
    vout_display_t *vd = picture->p_sys->vd;
    vout_display_sys_t *sys = vd->sys;
    picture_sys_t *pic_sys = picture->p_sys;
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_wait(sys->pool->queue);

    vlc_mutex_lock(&sys->buffer_mutex);

    mmal_buffer_header_reset(buffer);
    buffer->user_data = picture;
    pic_sys->buffer = buffer;

    memcpy(picture->p, sys->planes, sizeof(sys->planes));
    picture->p[0].p_pixels = buffer->data;
    picture->p[1].p_pixels += (ptrdiff_t)buffer->data;
    picture->p[2].p_pixels += (ptrdiff_t)buffer->data;

    pic_sys->displayed = false;

    vlc_mutex_unlock(&sys->buffer_mutex);

    return VLC_SUCCESS;
}

static void picture_unlock(picture_t *picture)
{
    picture_sys_t *pic_sys = picture->p_sys;
    vout_display_t *vd = pic_sys->vd;
    vout_display_sys_t *sys = vd->sys;
    MMAL_BUFFER_HEADER_T *buffer = pic_sys->buffer;

    vlc_mutex_lock(&sys->buffer_mutex);

    pic_sys->buffer = NULL;
    if (buffer) {
        buffer->user_data = NULL;
        mmal_buffer_header_release(buffer);
    }

    vlc_mutex_unlock(&sys->buffer_mutex);
}

static void control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    vout_display_t *vd = (vout_display_t *)port->userdata;
    MMAL_STATUS_T status;

    if (buffer->cmd == MMAL_EVENT_ERROR) {
        status = *(uint32_t *)buffer->data;
        msg_Err(vd, "MMAL error %"PRIx32" \"%s\"", status, mmal_status_to_string(status));
    }

    mmal_buffer_header_release(buffer);
}

static void input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    vout_display_t *vd = (vout_display_t *)port->userdata;
    vout_display_sys_t *sys = vd->sys;
    picture_t *picture = (picture_t *)buffer->user_data;

    vlc_mutex_lock(&sys->buffer_mutex);
    --sys->buffers_in_transit;
    vlc_cond_signal(&sys->buffer_cond);
    vlc_mutex_unlock(&sys->buffer_mutex);

    if (picture)
        picture_Release(picture);
}

static void* pool_allocator_alloc(void *context, uint32_t size)
{
    vout_display_t *vd = (vout_display_t *)context;
    vout_display_sys_t *sys = vd->sys;

    void* ret = mmal_port_payload_alloc(sys->input, size);
    return ret;
}

static void pool_allocator_free(void *context, void *mem)
{
    vout_display_t *vd = (vout_display_t *)context;
    vout_display_sys_t *sys = vd->sys;

    mmal_port_payload_free(sys->input, (uint8_t *)mem);
}

static int query_resolution(vout_display_t *vd, unsigned *width, unsigned *height)
{
    TV_DISPLAY_STATE_T display_state;
    int ret = 0;

    if (vc_tv_get_display_state(&display_state) == 0) {
        if (display_state.state & 0xFF) {
            *width = display_state.display.hdmi.width;
            *height = display_state.display.hdmi.height;
        } else if (display_state.state & 0xFF00) {
            *width = display_state.display.sdtv.width;
            *height = display_state.display.sdtv.height;
        }
        else {
            msg_Warn(vd, "Invalid display state %"PRIx32, display_state.state);
            ret = -1;
        }
    }
    else {
        msg_Warn(vd, "Failed to query display resolution");
        ret = -1;
    }

    return ret;
}

static void tvservice_cb(void *callback_data, uint32_t reason, uint32_t param1, uint32_t param2)
{
    VLC_UNUSED(reason);
    VLC_UNUSED(param1);
    VLC_UNUSED(param2);

    vout_display_t *vd = (vout_display_t *)callback_data;
    vout_display_sys_t *sys = vd->sys;

    vlc_mutex_lock(&sys->manage_mutex);
    sys->need_configure_display = true;
    vlc_mutex_unlock(&sys->manage_mutex);
}
