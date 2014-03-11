/*****************************************************************************
 * mmal.c: MMAL-based decoder plugin for Raspberry Pi
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
#include <vlc_codec.h>
#include <vlc_threads.h>

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>

static int OpenDecoder(decoder_t *dec);
static void CloseDecoder(decoder_t *dec);

#define MMAL_OPAQUE_MAGIC VLC_FOURCC('M', 'M', 'A', 'L')

#define MMAL_OPAQUE_NAME "mmal-opaque"
#define MMAL_OPAQUE_TEXT N_("Use opaque pointers")
#define MMAL_OPAQUE_LONGTEXT N_("Use opaque pointers")

vlc_module_begin()
    set_shortname(N_("MMAL decoder"))
    set_description(N_("MMAL-based decoder plugin for Raspberry Pi"))
    set_capability("decoder", 0)
    add_shortcut("mmal_decoder")
    add_bool(MMAL_OPAQUE_NAME, false, MMAL_OPAQUE_TEXT, MMAL_OPAQUE_LONGTEXT, false)
    set_callbacks(OpenDecoder, CloseDecoder)
vlc_module_end()

struct decoder_sys_t {
    bool opaque;
    MMAL_COMPONENT_T *component;
    MMAL_PORT_T *input;
    MMAL_POOL_T *input_pool;
    MMAL_PORT_T *output;
    MMAL_POOL_T *output_pool;
    MMAL_ES_FORMAT_T *output_format;
    MMAL_QUEUE_T *decoded_pictures;
    unsigned max_buffers_in_transit;
    vlc_mutex_t mutex;
};

/* Utilities */
static int change_output_format(decoder_t *dec);
static int send_output_buffer(decoder_t *dec);

/* VLC decoder callback */
static picture_t *decode(decoder_t *dec, block_t **block);

/* MMAL callbacks */
static void control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

static void* pool_allocator_alloc(void *context, uint32_t size);
static void pool_allocator_free(void *context, void *mem);

static int OpenDecoder(decoder_t *dec)
{
    int ret = VLC_SUCCESS;
    decoder_sys_t *sys;
    MMAL_STATUS_T status;

    if (dec->fmt_in.i_cat != VIDEO_ES) {
        ret = VLC_EGENERIC;
        goto out;
    }

    if (dec->fmt_in.i_codec != VLC_CODEC_MPGV && dec->fmt_in.i_codec != VLC_CODEC_H264) {
        ret = VLC_EGENERIC;
        goto out;
    }

    sys = calloc(1, sizeof(decoder_sys_t));
    if (!sys) {
        ret = VLC_ENOMEM;
        goto out;
    }
    dec->p_sys = sys;

    bcm_host_init();

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &sys->component);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to create MMAL component %s (status=%"PRIx32" %s)", MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->component->control->userdata = (struct MMAL_PORT_USERDATA_T *)dec;
    status = mmal_port_enable(sys->component->control, control_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to enable control port %s (status=%"PRIx32" %s)", sys->component->control->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->input = sys->component->input[0];
    sys->input->userdata = (struct MMAL_PORT_USERDATA_T *)dec;
    if (dec->fmt_in.i_codec == VLC_CODEC_MPGV)
        sys->input->format->encoding = MMAL_ENCODING_MP2V;
    else
        sys->input->format->encoding = MMAL_ENCODING_H264;

    if (dec->fmt_in.i_codec == VLC_CODEC_H264 && dec->fmt_in.i_extra > 0) {
        status = mmal_format_extradata_alloc(sys->input->format, dec->fmt_in.i_extra);
        if (status == MMAL_SUCCESS) {
            memcpy(sys->input->format->extradata, dec->fmt_in.p_extra, dec->fmt_in.i_extra);
            sys->input->format->extradata_size = dec->fmt_in.i_extra;
        }
        else {
            msg_Err(dec, "Failed to allocate extra format data on input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
        }
    }

    status = mmal_port_format_commit(sys->input);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to commit format for input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }
    sys->input->buffer_size = sys->input->buffer_size_recommended;
    sys->input->buffer_num = sys->input->buffer_num_recommended;

    status = mmal_port_enable(sys->input, input_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to enable input port %s (status=%"PRIx32" %s)", sys->input->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->output = sys->component->output[0];
    sys->output->userdata = (struct MMAL_PORT_USERDATA_T *)dec;

    status = mmal_port_enable(sys->output, output_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to enable output port %s (status=%"PRIx32" %s)", sys->output->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    status = mmal_component_enable(sys->component);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to enable component %s (status=%"PRIx32" %s)", sys->component->name, status, mmal_status_to_string(status));
        ret = VLC_EGENERIC;
        goto out;
    }

    sys->opaque = var_InheritBool(dec, MMAL_OPAQUE_NAME);
    sys->input_pool = mmal_pool_create_with_allocator(sys->input->buffer_num, sys->input->buffer_size, sys->input, pool_allocator_alloc, pool_allocator_free);
    sys->decoded_pictures = mmal_queue_create();

    if (sys->opaque) {
        sys->max_buffers_in_transit = 20;
        dec->i_extra_picture_buffers = 20;
    }
    else {
        sys->max_buffers_in_transit = 4;
    }

    dec->fmt_out.i_cat = VIDEO_ES;
    dec->pf_decode_video = decode;

    vlc_mutex_init(&sys->mutex);

out:
    if (ret != VLC_SUCCESS)
        CloseDecoder(dec);

    return ret;
}

static void CloseDecoder(decoder_t *dec)
{
    decoder_sys_t *sys = dec->p_sys;

    if (!sys)
        return;

    if (sys->component) {
        if (sys->component->is_enabled)
            mmal_component_disable(sys->component);

        if (sys->component->control->is_enabled)
            mmal_port_disable(sys->component->control);
    }

    if (sys->input && sys->input->is_enabled)
        mmal_port_disable(sys->input);

    if (sys->output && sys->output->is_enabled)
        mmal_port_disable(sys->output);

    if (sys->input_pool)
        mmal_pool_destroy(sys->input_pool);

    if (sys->output_format)
        mmal_format_free(sys->output_format);

    if (sys->decoded_pictures)
        mmal_queue_destroy(sys->decoded_pictures);

    if (sys->output_pool)
        mmal_pool_destroy(sys->output_pool);

    if (sys->component)
        mmal_component_release(sys->component);

    free(sys);

    bcm_host_deinit();
}

static int change_output_format(decoder_t *dec)
{
    decoder_sys_t *sys = dec->p_sys;
    MMAL_STATUS_T status;
    int ret = 0;

    status = mmal_port_disable(sys->output);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to disable output port (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        ret = -1;
        goto out;
    }

    mmal_format_full_copy(sys->output->format, sys->output_format);
    status = mmal_port_format_commit(sys->output);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to commit output format (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        ret = -1;
        goto out;
    }

    sys->output->buffer_num = sys->output->buffer_num_recommended;
    if (sys->output->buffer_num < sys->max_buffers_in_transit)
        sys->output->buffer_num = sys->max_buffers_in_transit;

    sys->output->buffer_size = sys->output->buffer_size_recommended;
    status = mmal_port_enable(sys->output, output_port_cb);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to enable output port (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        ret = -1;
        goto out;
    }

    sys->output_pool = mmal_pool_create(sys->output->buffer_num, 0);

    if (sys->opaque) {
        dec->fmt_out.i_codec = MMAL_OPAQUE_MAGIC;
        dec->fmt_out.video.i_chroma = MMAL_OPAQUE_MAGIC;
    }
    else {
        dec->fmt_out.i_codec = VLC_CODEC_I420;
        dec->fmt_out.video.i_chroma = VLC_CODEC_I420;
    }
    dec->fmt_out.video.i_width = sys->output->format->es->video.width;
    dec->fmt_out.video.i_height = sys->output->format->es->video.height;
    dec->fmt_out.video.i_x_offset = sys->output->format->es->video.crop.x;
    dec->fmt_out.video.i_y_offset = sys->output->format->es->video.crop.y;
    dec->fmt_out.video.i_visible_width = sys->output->format->es->video.crop.width;
    dec->fmt_out.video.i_visible_height = sys->output->format->es->video.crop.height;
    dec->fmt_out.video.i_sar_num = sys->output->format->es->video.par.num;
    dec->fmt_out.video.i_sar_den = sys->output->format->es->video.par.den;
    dec->fmt_out.video.i_frame_rate = sys->output->format->es->video.frame_rate.num;
    dec->fmt_out.video.i_frame_rate_base = sys->output->format->es->video.frame_rate.den;

out:
    return ret;
}

static int send_output_buffer(decoder_t *dec)
{
    decoder_sys_t *sys = dec->p_sys;
    MMAL_BUFFER_HEADER_T *buffer;
    picture_t *picture;
    MMAL_STATUS_T status;
    int ret = 0;

    buffer = mmal_queue_get(sys->output_pool->queue);
    if (!buffer) {
        msg_Warn(dec, "Failed to get new buffer");
        ret = -1;
        goto out;
    }

    picture = decoder_NewPicture(dec);
    if (!picture) {
        msg_Warn(dec, "Failed to get new picture");
        mmal_buffer_header_release(buffer);
        ret = -1;
        goto out;
    }

    mmal_buffer_header_reset(buffer);
    buffer->user_data = picture;
    buffer->cmd = 0;
    buffer->alloc_size = sys->output->buffer_size;
    buffer->data = picture->p[0].p_pixels;

    status = mmal_port_send_buffer(sys->output, buffer);
    if (status != MMAL_SUCCESS) {
        msg_Err(dec, "Failed to send buffer to output port (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
        mmal_buffer_header_release(buffer);
        decoder_DeletePicture(dec, picture);
        ret = -1;
        goto out;
    }

out:
    return ret;
}

static picture_t *decode(decoder_t *dec, block_t **pblock)
{
    decoder_sys_t *sys = dec->p_sys;
    block_t *block;
    MMAL_BUFFER_HEADER_T *buffer;
    uint32_t len;
    uint32_t flags = 0;
    picture_t *picture;
    MMAL_STATUS_T status;
    picture_t *ret = NULL;

    /*
     * Configure output port if necessary
     */
    if (sys->output_format && !sys->output_pool)
        if (change_output_format(dec) < 0)
            msg_Err(dec, "Failed to change output port format");

    /*
     * Send output buffers
     */
    if (sys->output_pool) {
        buffer = mmal_queue_get(sys->decoded_pictures);
        if (buffer) {
            picture = (picture_t *)buffer->user_data;

            if (buffer->length > 0) {
                picture->date = buffer->pts;
                ret = picture;
            }
            else {
                decoder_DeletePicture(dec, picture);
            }

            buffer->user_data = NULL;
            buffer->alloc_size = 0;
            buffer->data = NULL;
            mmal_buffer_header_release(buffer);
        }

        vlc_mutex_lock(&sys->mutex);
        while (mmal_queue_length(sys->output_pool->queue) > (sys->output->buffer_num - sys->max_buffers_in_transit))
            if (send_output_buffer(dec) < 0)
                break;
        vlc_mutex_unlock(&sys->mutex);
    }

    /*
     * Process input
     */
    if (!pblock)
        goto out;

    block = *pblock;

    if (!block)
        goto out;

    *pblock = NULL;

    if (block->i_flags & BLOCK_FLAG_CORRUPTED)
        flags |= MMAL_BUFFER_HEADER_FLAG_CORRUPTED;

    if (block->i_flags & BLOCK_FLAG_DISCONTINUITY)
        flags |= MMAL_BUFFER_HEADER_FLAG_DISCONTINUITY;

    while (block->i_buffer > 0) {
        buffer = mmal_queue_wait(sys->input_pool->queue);
        mmal_buffer_header_reset(buffer);
        buffer->cmd = 0;
        buffer->pts = block->i_pts;
        buffer->dts = block->i_dts;

        len = block->i_buffer;
        if (len > buffer->alloc_size)
            len = buffer->alloc_size;

        memcpy(buffer->data, block->p_buffer, len);
        block->p_buffer += len;
        block->i_buffer -= len;

        buffer->length = len;
        buffer->flags = flags;

        status = mmal_port_send_buffer(sys->input, buffer);
        if (status != MMAL_SUCCESS) {
            msg_Err(dec, "Failed to send buffer to input port (status=%"PRIx32" %s)", status, mmal_status_to_string(status));
            break;
        }
    }

    block_Release(block);

out:
    return ret;
}

static void control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    decoder_t *dec = (decoder_t *)port->userdata;
    MMAL_STATUS_T status;

    if (buffer->cmd == MMAL_EVENT_ERROR) {
        status = *(uint32_t *)buffer->data;
        msg_Err(dec, "MMAL error %"PRIx32" \"%s\"", status, mmal_status_to_string(status));
    }

    mmal_buffer_header_release(buffer);
}

static void input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    VLC_UNUSED(port);

    mmal_buffer_header_release(buffer);
}

static void output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    decoder_t *dec = (decoder_t *)port->userdata;
    decoder_sys_t *sys = dec->p_sys;
    MMAL_EVENT_FORMAT_CHANGED_T *fmt;
    MMAL_ES_FORMAT_T *format;

    if (buffer->cmd == 0) {
        mmal_queue_put(sys->decoded_pictures, buffer);

        int ii;
        printf("dec : ");
        for (ii = 0; ii < 16; ++ii)
            printf("%"PRIx32" ", ((uint32_t*)buffer->data)[ii]);
        printf("\n");

        vlc_mutex_lock(&sys->mutex);
        while (mmal_queue_length(sys->output_pool->queue) > (sys->output->buffer_num - sys->max_buffers_in_transit))
            if (send_output_buffer(dec) < 0)
                break;
        vlc_mutex_unlock(&sys->mutex);
    }
    else if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED) {
        fmt = mmal_event_format_changed_get(buffer);

        format = mmal_format_alloc();
        mmal_format_full_copy(format, fmt->format);

        if (sys->opaque)
            format->encoding = MMAL_ENCODING_OPAQUE;

        sys->output_format = format;

        mmal_buffer_header_release(buffer);
    }
    else if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED) {
        mmal_buffer_header_release(buffer);
    }
}

static void* pool_allocator_alloc(void *context, uint32_t size)
{
    return mmal_port_payload_alloc((MMAL_PORT_T *)context, size);
}

static void pool_allocator_free(void *context, void *mem)
{
    mmal_port_payload_free((MMAL_PORT_T *)context, (uint8_t *)mem);
}