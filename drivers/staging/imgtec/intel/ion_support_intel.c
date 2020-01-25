/*************************************************************************/ /*!
@File           ion_support_intel.c
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

#include "private_data.h"
#include "pvrsrv_error.h"
#include "ion_support.h"
#include "dc_server.h"
#include "ion_sys.h"
#include "pmr.h"

#include PVR_ANDROID_ION_HEADER
#include PVR_ANDROID_ION_PRIV_HEADER

#include <linux/scatterlist.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/file.h>

#include <asm/uaccess.h>

static struct ion_device *gpsIonDev;

/* BEGIN Keep in sync with buffer_intel.c in usermode */

struct ion_allocation_dc_private_data
{
	ion_user_handle_t handle;
	__u64 dc_buffer;
	__s32 fd;
} __packed;

#define ION_IOC_ALLOC_FROM_DC_BUFFER \
	_IOWR(ION_IOC_MAGIC, 0, struct ion_allocation_dc_private_data)

/* END-- Keep in sync with buffer_intel.c in usermode */

/* BEGIN Keep in sync with ion.c */

struct ion_handle {
	struct kref ref;
	struct ion_client *client;
	struct ion_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	int id;
};

/* END-- Keep in sync with ion.c */

/* Protected by the bridge lock. Non-NULL during custom ioctl. */
static PMR *gpsDCBufferPMR;

static int DCHeapAllocate(struct ion_heap *psHeap,
						  struct ion_buffer *psBuffer,
						  unsigned long ulLen, unsigned long ulAlign,
						  unsigned long ulFlags)
{
	struct scatterlist *psScatterList;
	IMG_DEV_PHYADDR *pasDevAddr;
	struct sg_table *psTable;
	struct page **psPages;
	size_t i, u32NumPages;
	PVRSRV_ERROR eError;
	IMG_BOOL *pabValid;
	int err = -ENOMEM;

	PVR_UNREFERENCED_PARAMETER(psHeap);
	PVR_UNREFERENCED_PARAMETER(ulAlign);
	PVR_UNREFERENCED_PARAMETER(ulFlags);

	/* It's not really correct to hijack the PMR lock like this, but this
	 * seems better than relying on the bridge lock since that could be
	 * taken around a call to ion_alloc(). The PMR lock being held around
	 * a call to ion_alloc() is much less likely.
	 */
	if (!PMRIsLockedByMe() || !gpsDCBufferPMR)
	{
		err = -ENOTTY;
		goto err_out;
	}

	eError = PMRLockSysPhysAddresses(gpsDCBufferPMR, PAGE_SHIFT);
	if (eError != PVRSRV_OK)
	{
		err = -EFAULT;
		goto err_out;
	}

	u32NumPages = ulLen >> PAGE_SHIFT;

	pasDevAddr = vzalloc(sizeof(IMG_DEV_PHYADDR) * u32NumPages);
	if (!pasDevAddr)
		goto err_unlock;

	pabValid = vzalloc(sizeof(IMG_BOOL) * u32NumPages);
	if (!pabValid)
		goto err_free_devaddr;

	psPages = vzalloc(sizeof(struct page *) * u32NumPages);
	if (!psPages)
		goto err_free_valid;

	psTable = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!psTable)
		goto err_free_pages;

	err = sg_alloc_table(psTable, u32NumPages, GFP_KERNEL);
	if (err)
		goto err_free_sg_table;

	eError = PMR_DevPhysAddr(gpsDCBufferPMR, PAGE_SHIFT, u32NumPages, 0,
							 pasDevAddr, pabValid);
	if (eError != PVRSRV_OK)
	{
		err = -EFAULT;
		goto err_free_table;
	}

	for (i = 0; i < u32NumPages; i++)
	{
		if (!pabValid[i])
		{
			err = -EFAULT;
			goto err_free_table;
		}

		psPages[i] = pfn_to_page(PFN_DOWN(pasDevAddr[i].uiAddr));
	}

	for_each_sg(psTable->sgl, psScatterList, psTable->nents, i)
	{
		sg_set_page(psScatterList, psPages[i], PAGE_SIZE, 0);
		get_page(psPages[i]);
	}

	psBuffer->priv_virt = psTable;
	psBuffer->pages = psPages;
	err = 0;
err_free_valid:
	vfree(pabValid);
err_free_devaddr:
	vfree(pasDevAddr);
err_unlock:
	PMRUnlockSysPhysAddresses(gpsDCBufferPMR);
err_out:
	return err;

err_free_table:
	sg_free_table(psTable);
err_free_sg_table:
	kfree(psTable);
err_free_pages:
	vfree(psPages);
	goto err_free_valid;
}

static void DCHeapFree(struct ion_buffer *psBuffer)
{
	struct sg_table *psTable = psBuffer->priv_virt;
	struct scatterlist *psScatterList;
	size_t i;

	for_each_sg(psTable->sgl, psScatterList, psTable->nents, i)
		put_page(psBuffer->pages[i]);

	sg_free_table(psTable);
	kfree(psTable);
}

static struct sg_table *DCHeapMapDMA(struct ion_heap *psHeap,
									 struct ion_buffer *psBuffer)
{
	PVR_UNREFERENCED_PARAMETER(psHeap);

	return psBuffer->priv_virt;
}

static void DCHeapUnmapDMA(struct ion_heap *psHeap,
						   struct ion_buffer *psBuffer)
{
	PVR_UNREFERENCED_PARAMETER(psHeap);
	PVR_UNREFERENCED_PARAMETER(psBuffer);
}

static void *DCHeapMapKernel(struct ion_heap *psHeap,
							 struct ion_buffer *psBuffer)
{
	struct sg_table *psTable = psBuffer->priv_virt;
	void *pvAddr;

	PVR_UNREFERENCED_PARAMETER(psHeap);

	pvAddr = vmap(psBuffer->pages, psTable->nents, VM_MAP,
				  __pgprot((pgprot_val(PAGE_KERNEL) & ~_PAGE_CACHE_MASK) |
						   _PAGE_CACHE_WC));
	if (!pvAddr)
		return ERR_PTR(-ENOMEM);

	return pvAddr;
}

static void DCHeapUnmapKernel(struct ion_heap *psHeap,
							  struct ion_buffer *psBuffer)
{
	PVR_UNREFERENCED_PARAMETER(psHeap);

	vunmap(psBuffer->vaddr);
}

static int DCHeapMapUser(struct ion_heap *psHeap, struct ion_buffer *psBuffer,
						 struct vm_area_struct *psVMA)
{
	unsigned long ulOffset = psVMA->vm_pgoff * PAGE_SIZE;
	struct sg_table *psTable = psBuffer->priv_virt;
	unsigned long ulAddr = psVMA->vm_start;
	struct scatterlist *psScatterList;
	int i, err;

	for_each_sg(psTable->sgl, psScatterList, psTable->nents, i)
	{
		unsigned long ulRemainder = psVMA->vm_end - ulAddr;
		unsigned long ulLen = psScatterList->length;
		struct page *psPage = psBuffer->pages[i];

		if (ulOffset >= psScatterList->length)
		{
			ulOffset -= psScatterList->length;
			continue;
		}

		if (ulOffset)
		{
			psPage += ulOffset / PAGE_SIZE;
			ulLen = psScatterList->length - ulOffset;
			ulOffset = 0;
		}

		ulLen = min(ulLen, ulRemainder);

		err = remap_pfn_range(psVMA, ulAddr, page_to_pfn(psPage),
							  ulLen, psVMA->vm_page_prot);
		if (err)
			goto err_out;

		ulAddr += ulLen;
		if (ulAddr >= psVMA->vm_end)
			break;
	}

	err = 0;
err_out:
	return err;
}

static struct ion_heap_ops gsDCHeapOps =
{
	.allocate = DCHeapAllocate,
	.free = DCHeapFree,
	.map_dma = DCHeapMapDMA,
	.unmap_dma = DCHeapUnmapDMA,
	.map_kernel = DCHeapMapKernel,
	.unmap_kernel = DCHeapUnmapKernel,
	.map_user = DCHeapMapUser,
};

static struct ion_heap gsDCHeap =
{
	.type = ION_HEAP_TYPE_CUSTOM,
	.name = "dc",
	.id = ION_HEAP_TYPE_CUSTOM + 0,
	.ops = &gsDCHeapOps,
};

static struct ion_platform_heap gsSystemHeap =
{
	.type = ION_HEAP_TYPE_SYSTEM,
	.name = "system",
	.id = ION_HEAP_TYPE_SYSTEM,
};

static struct ion_heap *gpsSystemHeap;

static long IonCustomIoctl(struct ion_client *psClient,
						   unsigned int uiCmd, unsigned long ulArg)
{
	struct ion_allocation_dc_private_data sData;
	CONNECTION_DATA *psConnection;
	struct ion_handle *psHandle;
	IMG_DEVMEM_SIZE_T uSize;
	struct file *psFile;
	PVRSRV_ERROR eError;
	DC_BUFFER *psBuffer;
	int err = -EFAULT;

	if (uiCmd != ION_IOC_ALLOC_FROM_DC_BUFFER)
	{
		err = -ENOTTY;
		goto err_out;
	}

	if (_IOC_SIZE(uiCmd) > sizeof(struct ion_allocation_dc_private_data))
	{
		err = -EINVAL;
		goto err_out;
	}

	if (copy_from_user(&sData, (void __user *)ulArg, _IOC_SIZE(uiCmd)))
		goto err_out;

	psFile = fget(sData.fd);
	if (!psFile)
		goto err_out;

	psConnection = LinuxConnectionFromFile(psFile);
	if (!psConnection)
		goto err_fput;

	OSAcquireBridgeLock();

	eError = PVRSRVLookupHandle(psConnection->psHandleBase,
								(void **)&psBuffer,
								(IMG_HANDLE)sData.dc_buffer,
								PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (eError != PVRSRV_OK)
		goto err_unlock;

	PMRLock();

	eError = DCBufferAcquire(psBuffer, &gpsDCBufferPMR);
	if (eError != PVRSRV_OK)
		goto err_unlock_pmr;

	eError = PMR_LogicalSize(gpsDCBufferPMR, &uSize);
	if (eError != PVRSRV_OK)
		goto err_unlock_pmr;

	/* Ion has no means of passing private data down into the alloc function,
	 * so pass gpsDCBufferPMR as global data. This pointer will only be valid
	 * when the bridge lock is held, so the allocation function can only be
	 * used by the custom ioctl interface.
	 */

	psHandle = ion_alloc(psClient, uSize, PAGE_SIZE, 1 << gsDCHeap.id, 0);
	if (IS_ERR(psHandle))
	{
		err = PTR_ERR(psHandle);
		goto err_unlock_pmr;
	}

	sData.handle = psHandle->id;

	if (copy_to_user((void __user *)ulArg, &sData, _IOC_SIZE(uiCmd)))
		goto err_free;

	err = 0;
err_unlock_pmr:
	DCBufferRelease(gpsDCBufferPMR);
	PMRUnlock();
	gpsDCBufferPMR = NULL;
err_unlock:
	OSReleaseBridgeLock();
err_fput:
	fput(psFile);
err_out:
	return (long)err;

err_free:
	ion_free(psClient, psHandle);
	goto err_unlock_pmr;
}

PVRSRV_ERROR IonInit(void *phPrivateData)
{
	PVR_UNREFERENCED_PARAMETER(phPrivateData);

	gpsIonDev = ion_device_create(IonCustomIoctl);
	if (IS_ERR_OR_NULL(gpsIonDev))
		goto fail;

	gpsSystemHeap = ion_heap_create(&gsSystemHeap);
	if (IS_ERR_OR_NULL(gpsSystemHeap))
		goto failHeapCreate;

	ion_device_add_heap(gpsIonDev, gpsSystemHeap);
	ion_device_add_heap(gpsIonDev, &gsDCHeap);
	return PVRSRV_OK;

failHeapCreate:
	ion_device_destroy(gpsIonDev);
fail:
	return PVRSRV_ERROR_OUT_OF_MEMORY;
}

struct ion_device *IonDevAcquire(void)
{
	return gpsIonDev;
}

void IonDevRelease(struct ion_device *psIonDev)
{
	PVR_ASSERT(psIonDev == gpsIonDev);
}

void IonDeinit(void)
{
	if (gpsSystemHeap)
		ion_heap_destroy(gpsSystemHeap);

	ion_device_destroy(gpsIonDev);
}
