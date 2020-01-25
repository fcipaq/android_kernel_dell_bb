/*************************************************************************/ /*!
@File
@Title          Services DRM definitions shared between kernel and user space.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(__PVR_DRM_H__)
#define __PVR_DRM_H__

#include <linux/types.h>

#if defined(__KERNEL__)
#include <drm/drm.h>
#elif defined(SUPPORT_ANDROID_PLATFORM) && \
     !defined(PVR_ANDROID_OLD_LIBDRM_HEADER_PATH)
#include <drm.h>
#else
#include <libdrm/drm.h>
#endif

/*
 * The fields, types and ordering must match those of PVRSRV_BRIDGE_PACKAGE but
 * the names must match those of the common version of this structure found in a
 * header of the same name in the common Services code.
 */
struct drm_pvr_srvkm_cmd {
	__u32 bridge_id;
	__u32 bridge_func_id;
	__u32 size;
	void *in_data_ptr;
	__u32 in_data_size;
	void *out_data_ptr;
	__u32 out_data_size;
};

struct drm_pvr_dbgdrv_cmd {
	__u32 cmd;
	__u32 pad;
	__u64 in_data_ptr;
	__u64 out_data_ptr;
	__u32 in_data_size;
	__u32 out_data_size;
};

/* 
 * DRM command numbers, relative to DRM_COMMAND_BASE. 
 * These defines must be prefixed with "DRM_".
 */
#define DRM_PVR_SRVKM_CMD		0x12 /* Used for PVR Services ioctls */
#define DRM_PVR_IS_MASTER_CMD	0x15 /* Used to check for master */
#define DRM_PVR_DBGDRV_CMD		0x1E /* Debug driver (PDUMP) ioctls */

#define DRM_IOCTL_PVR_SRVKM_CMD \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_SRVKM_CMD, struct drm_pvr_srvkm_cmd)

#define DRM_IOCTL_PVR_IS_MASTER_CMD \
	DRM_IO(DRM_COMMAND_BASE + DRM_PVR_IS_MASTER_CMD)

#define	DRM_IOCTL_PVR_DBGDRV_CMD \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_DBGDRV_CMD, struct drm_pvr_dbgdrv_cmd)

#endif /* defined(__PVR_DRM_H__) */
