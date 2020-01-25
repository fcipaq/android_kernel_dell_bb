/*************************************************************************/ /*!                                                                                                               
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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <drm/drmP.h>
#include <drm/drm.h>

#include "img_defs.h"
#include "lock.h"
#include "pvr_drm_ext.h"
#include "pvr_drm.h"
#include "pvrsrv_ext.h"
#include "pvrsrv_interface.h"
#include "srvkm.h"
#include "dc_mrfld.h"
#include "drm_shared.h"
#include "module_common.h"

#include <linux/module.h>
#include "pvrmodule.h"

#if (DRM_PVR_RESERVED1 != DRM_PVR_SRVKM_CMD) ||	\
	(DRM_PVR_RESERVED4 != DRM_PVR_IS_MASTER_CMD) || \
	(DRM_PVR_RESERVED6 != DRM_PVR_DBGDRV_CMD)
#error Mismatch in IOCTL numbers
#endif


static int
PVRDRMIsMaster(struct drm_device *dev, void *arg, struct drm_file *pFile)
{
	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
static struct drm_ioctl_desc pvr_ioctls[] = {
	{DRM_IOCTL_PVR_SRVKM_CMD, DRM_UNLOCKED, PVRSRV_BridgeDispatchKM},
	{DRM_IOCTL_PVR_IS_MASTER_CMD, DRM_MASTER, PVRDRMIsMaster},
#if defined(PDUMP)
	{DRM_IOCTL_PVR_DBGDRV_CMD, 0, dbgdrv_ioctl}
#endif
};
#else
static struct drm_ioctl_desc pvr_ioctls[] = {
	{DRM_IOCTL_PVR_SRVKM_CMD, DRM_UNLOCKED, PVRSRV_BridgeDispatchKM, DRM_IOCTL_PVR_SRVKM_CMD},
	{DRM_IOCTL_PVR_IS_MASTER_CMD, DRM_MASTER, PVRDRMIsMaster, DRM_IOCTL_PVR_IS_MASTER_CMD},
#if defined(PDUMP)
	{DRM_IOCTL_PVR_DBGDRV_CMD, 0, dbgdrv_ioctl, DRM_IOCTL_PVR_DBGDRV_CMD}
#endif
};
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)) */

typedef struct _PVRSRV_DEVICE_NODE_ PVRSRV_DEVICE_NODE;

DECLARE_WAIT_QUEUE_HEAD(sWaitForInit);

static bool bInitComplete;
static bool bInitFailed;

static struct pci_dev *gpsPVRLDMDev;

struct drm_device *gpsPVRDRMDev;
static PVRSRV_DEVICE_NODE *gpsDeviceNode;

#define PVR_DRM_FILE struct drm_file *

int PVRCore_Init(void)
{
	int error = 0;

	if ((error = PVRSRVCommonDriverInit()) != 0)
	{
		return error;
	}

	error = PVRSRVDeviceCreate(&gpsPVRLDMDev->dev, &gpsDeviceNode);
	if (error != 0)
	{
		DRM_DEBUG("%s: unable to init PVR service (%d)", __FUNCTION__, error);
		return error;
	}

	error = PVRSRVCommonDeviceInit(gpsDeviceNode);
	if (error != 0)
	{
		return error;
	}

	return 0;
}

void PVRCore_Cleanup(void)
{
	PVRSRVCommonDeviceDeinit(gpsDeviceNode);
	PVRSRVDeviceDestroy(gpsDeviceNode);
	gpsDeviceNode = NULL;

	PVRSRVCommonDriverDeinit();
}

int PVRSRVOpen(struct drm_device __maybe_unused *dev, struct drm_file *pDRMFile)
{
	int err;

	if (!try_module_get(THIS_MODULE))
	{
		DRM_DEBUG("%s: Failed to get module", __FUNCTION__);
		return -ENOENT;
	}

	err = PVRSRVCommonDeviceOpen(gpsDeviceNode, pDRMFile);
	if (err)
	{
		module_put(THIS_MODULE);
	}

	return err;
}

void PVRSRVRelease(struct drm_device __maybe_unused *dev, struct drm_file *pDRMFile)
{
	PVRSRVCommonDeviceRelease(gpsDeviceNode, pDRMFile);

	module_put(THIS_MODULE);
}

int PVRSRVDrmLoad(struct drm_device *dev, unsigned long flags)
{
	int iRes = 0;

	DRM_DEBUG("PVRSRVDrmLoad");

	gpsPVRDRMDev = dev;
	gpsPVRLDMDev = dev->pdev;
	
	iRes = PVRCore_Init();
	if (iRes != 0)
	{
		goto exit;
	}

	if (MerrifieldDCInit(dev) != PVRSRV_OK)
	{
		DRM_ERROR("%s: display class init failed\n", __FUNCTION__);
		goto exit_pvrcore_cleanup;
	}

	goto exit;

exit_pvrcore_cleanup:
	PVRCore_Cleanup();
exit:
	if (iRes != 0)
	{
		bInitFailed = true;
	}
	bInitComplete = true;

	wake_up_interruptible(&sWaitForInit);

	return iRes;
}

int PVRSRVDrmUnload(struct drm_device *dev)
{
	DRM_DEBUG("PVRSRVDrmUnload");

	if (MerrifieldDCDeinit() != PVRSRV_OK)
	{
		DRM_ERROR("%s: can't deinit display class\n", __FUNCTION__);
	}

	PVRCore_Cleanup();

	return 0;
}

int PVRSRVDrmOpen(struct drm_device *dev, struct drm_file *file)
{
	while (!bInitComplete)
	{
		DEFINE_WAIT(sWait);

		prepare_to_wait(&sWaitForInit, &sWait, TASK_INTERRUPTIBLE);

		if (!bInitComplete)
		{
			DRM_DEBUG("%s: Waiting for module initialisation to complete", __FUNCTION__);

			schedule();
		}

		finish_wait(&sWaitForInit, &sWait);

		if (signal_pending(current))
		{
			return -ERESTARTSYS;
		}
	}

	if (bInitFailed)
	{
		DRM_DEBUG("%s: Module initialisation failed", __FUNCTION__);
		return -EINVAL;
	}

	return PVRSRVOpen(dev, file);
}

void PVRSRVDrmPostClose(struct drm_device *dev, struct drm_file *file)
{
	PVRSRVRelease(dev, file);

	file->driver_priv = NULL;
}

void PVRSRVQueryIoctls(struct drm_ioctl_desc *ioctls)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pvr_ioctls); i++)
	{
		unsigned int slot = DRM_IOCTL_NR(pvr_ioctls[i].cmd) - DRM_COMMAND_BASE;
		ioctls[slot] = pvr_ioctls[i];
	}
}

unsigned int PVRSRVGetMeminfoSize(void* hMemHandle)
{
    PVRSRV_MEMINFO  minfo;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    return minfo.uiAllocationSize;
}

void * PVRSRVGetMeminfoCPUAddr(void* hMemHandle)
{
    PVRSRV_MEMINFO  minfo;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    return minfo.pvCpuVirtAddr;
}

int PVRSRVGetMeminfoPages(void* hMemHandle, int npages, struct page ***pages)
{
    PVRSRV_MEMINFO  minfo;
    struct page   **pglist;
    uint32_t        kaddr;
    int             res;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return -EFAULT;
    }

    kaddr = (uint32_t)(uintptr_t)minfo.pvCpuVirtAddr;

    if ((pglist = kzalloc(npages * sizeof(struct page*),GFP_KERNEL)) == NULL)
    {
        return -ENOMEM;
    }

    down_read(&current->mm->mmap_sem);
    res = get_user_pages(current,current->mm,kaddr,npages,0,0,pglist,NULL);
    up_read(&current->mm->mmap_sem);

    if (res <= 0)
    {
        kfree(pglist);
        return res;
    }

    *pages = pglist;
	return 0;
}

int PVRSRVGetMeminfoPfn(void           *hMemHandle,
                        int             npages,
                        unsigned long **pfns)
{
    PVRSRV_MEMINFO          minfo;
    struct vm_area_struct  *vma;
    unsigned long          *pfnlist;
    uint32_t                kaddr;
    int                     res, pg = 0;

    /*
     *  This 'handle' is a pointer in user space to a meminfo struct.
     *  We need to copy it here and get the user's view of memory.
     */
    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    kaddr = (uint32_t)(uintptr_t)minfo.pvCpuVirtAddr;

    if ((pfnlist = kzalloc(npages * sizeof(unsigned long),
                           GFP_KERNEL)) == NULL)
    {
        return -ENOMEM;
    }

    while (pg < npages)
    {
        if ((vma = find_vma(current->mm,
                            kaddr + (pg * PAGE_SIZE))) == NULL)
        {
            kfree(pfnlist);
            return -EFAULT;
        }

        if ((res = follow_pfn(
                        vma,
                        (unsigned long)(kaddr + (pg * PAGE_SIZE)),
                        &pfnlist[pg])) < 0)
        {
            kfree(pfnlist);
            return res;
        }

        ++pg;
    }

    *pfns = pfnlist;
    return 0;
}

int PVRSRVInterrupt(struct drm_device* dev)
{
	return 1;
}

int PVRSRVMMap(struct file *pFile, struct vm_area_struct *ps_vma)
{
	return PVRSRV_MMap(pFile, ps_vma);
}

PVRSRV_ERROR PVRSRVEnumerateDevicesKM(IMG_UINT32 *pui32NumDevices,
									  PVRSRV_DEVICE_TYPE *peDeviceType,
									  PVRSRV_DEVICE_CLASS *peDeviceClass,
									  IMG_UINT32 *pui32DeviceIndex)
{
	IMG_UINT32 i;

	if (!pui32NumDevices || !peDeviceType || !peDeviceClass || !pui32DeviceIndex)
	{
		DRM_ERROR("%s: Invalid params", __func__);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Setup input buffer to be `empty' */
	for (i = 0; i < PVRSRV_MAX_DEVICES; i++)
	{
		peDeviceType[i] = PVRSRV_DEVICE_TYPE_UNKNOWN;
	}

	/* Only a single device is supported */
	peDeviceType[0] = PVRSRV_DEVICE_TYPE_RGX;
	peDeviceClass[0] = PVRSRV_DEVICE_CLASS_3D;
	pui32DeviceIndex[0] = 0;
	*pui32NumDevices = 1;

	return PVRSRV_OK;
}

PVRSRV_ERROR PVRSRVAcquireDeviceDataKM(IMG_UINT32 ui32DevIndex,
									   PVRSRV_DEVICE_TYPE eDeviceType,
									   IMG_HANDLE *phDevCookie)
{
	if (!phDevCookie)
	{
		DRM_ERROR("%s: Invalid params", __func__);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if ((eDeviceType == PVRSRV_DEVICE_TYPE_RGX) ||
		(eDeviceType == PVRSRV_DEVICE_TYPE_UNKNOWN && ui32DevIndex == 0))
	{
		PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

		/*
		 * Return the head of the list, which will only contain a single entry,
		 * as only a single device is supported.
		 */
		*phDevCookie = (IMG_HANDLE) psPVRSRVData->psDeviceNodeList;

		return PVRSRV_OK;
	}

	DRM_ERROR("%s: requested device is not present", __func__);
	return PVRSRV_ERROR_INIT_FAILURE;
}
