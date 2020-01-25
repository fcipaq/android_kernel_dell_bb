/*************************************************************************/ /*!
@File
@Title          Environment related functions
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

#include <linux/version.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/hugetlb.h> 
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/hardirq.h>
#include <asm/tlbflush.h>
#include <linux/timer.h>
#include <linux/capability.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE) || \
	defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) || \
	defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || \
	defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE) || \
	defined(PVR_LINUX_USING_WORKQUEUES)
#include <linux/workqueue.h>
#endif
#include <linux/kthread.h>
#include <asm/atomic.h>

#include "log2.h"
#include "osfunc.h"
#include "img_types.h"
#include "mm.h"
#include "allocmem.h"
#include "env_data.h"
#include "pvr_debugfs.h"
#include "event.h"
#include "linkage.h"
#include "pvr_uaccess.h"
#include "pvr_debug.h"
#include "driverlock.h"
#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#include "process_stats.h"
#endif
#if defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
#include "syscommon.h"
#endif
#include "physmem_osmem_linux.h"
#if defined(SUPPORT_PVRSRV_GPUVIRT)
#include "virt_support.h"
#endif

#if defined(VIRTUAL_PLATFORM)
#define EVENT_OBJECT_TIMEOUT_MS         (120000)
#else
#if defined(EMULATOR)
#define EVENT_OBJECT_TIMEOUT_MS		(2000)
#else
#define EVENT_OBJECT_TIMEOUT_MS		(100)
#endif /* EMULATOR */
#endif


static void *g_pvBridgeBuffers = NULL;
static atomic_t g_DriverSuspended;

struct task_struct *OSGetBridgeLockOwner(void);


PVRSRV_ERROR OSPhyContigPagesAlloc(PVRSRV_DEVICE_NODE *psDevNode, size_t uiSize,
							PG_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr)
{
	IMG_CPU_PHYADDR sCpuPAddr;
	struct page *psPage;
	IMG_UINT32	ui32Order=0;

	PVR_ASSERT(uiSize != 0);
	/*Align the size to the page granularity */
	uiSize = PAGE_ALIGN(uiSize);

	/*Get the order to be used with the allocation */
	ui32Order = get_order(uiSize);

	/*allocate the pages */
	psPage = alloc_pages(GFP_KERNEL, ui32Order);
	if (psPage == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	uiSize = (1 << ui32Order) * PAGE_SIZE;

	psMemHandle->u.pvHandle = psPage;
	psMemHandle->ui32Order = ui32Order;
	sCpuPAddr.uiAddr =  IMG_CAST_TO_CPUPHYADDR_UINT(page_to_phys(psPage));

	/*
	 * Even when more pages are allocated as base MMU object we still need one single physical address because
	 * they are physically contiguous.
	 */
	PhysHeapCpuPAddrToDevPAddr(psDevNode->apsPhysHeap[PVRSRV_DEVICE_PHYS_HEAP_CPU_LOCAL], 1, psDevPAddr, &sCpuPAddr);

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	    PVRSRVStatsIncrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, uiSize);
#else
	PVRSRVStatsAddMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA,
								 psPage,
								 sCpuPAddr,
								 uiSize,
								 NULL);
#endif
#endif

	return PVRSRV_OK;
}

void OSPhyContigPagesFree(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle)
{
	struct page *psPage = (struct page*) psMemHandle->u.pvHandle;
	IMG_UINT32	uiSize, uiPageCount=0;

	uiPageCount = (1 << psMemHandle->ui32Order);
	uiSize = (uiPageCount * PAGE_SIZE);

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	    PVRSRVStatsDecrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, uiSize);
#else
	PVRSRVStatsRemoveMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, (IMG_UINT64)(uintptr_t)psPage);
#endif
#endif

	__free_pages(psPage, psMemHandle->ui32Order);
	psMemHandle->ui32Order = 0;
}

PVRSRV_ERROR OSPhyContigPagesMap(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle,
						size_t uiSize, IMG_DEV_PHYADDR *psDevPAddr,
						void **pvPtr)
{
	size_t actualSize = 1<<psMemHandle->ui32Order;
	*pvPtr = kmap((struct page*)psMemHandle->u.pvHandle);

	PVR_UNREFERENCED_PARAMETER(psDevPAddr);

	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(psDevNode);

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	PVRSRVStatsIncrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, actualSize);
#else
	{
		IMG_CPU_PHYADDR sCpuPAddr;
		sCpuPAddr.uiAddr = 0;

		PVRSRVStatsAddMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA,
									 *pvPtr,
									 sCpuPAddr,
									 actualSize,
									 NULL);
	}
#endif
#endif

	return PVRSRV_OK;
}

void OSPhyContigPagesUnmap(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle, void *pvPtr)
{
#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	/* Mapping is done a page at a time */
	PVRSRVStatsDecrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, (PAGE_SIZE << psMemHandle->ui32Order));
#else
	PVRSRVStatsRemoveMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, (IMG_UINT64)(uintptr_t)pvPtr);
#endif
#endif

	PVR_UNREFERENCED_PARAMETER(psDevNode);
	PVR_UNREFERENCED_PARAMETER(pvPtr);

	kunmap((struct page*) psMemHandle->u.pvHandle);
}

PVRSRV_ERROR OSPhyContigPagesClean(PG_HANDLE *psMemHandle,
                                   IMG_UINT32 uiOffset,
                                   IMG_UINT32 uiLength)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	struct page* psPage = (struct page*) psMemHandle->u.pvHandle;

	void* pvVirtAddrStart = kmap(psPage) + uiOffset;
	IMG_CPU_PHYADDR sPhysStart, sPhysEnd;

	if (uiLength == 0)
	{
		goto e0;
	}

	if ((uiOffset + uiLength) > ((1 << psMemHandle->ui32Order) * PAGE_SIZE))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Invalid size params, uiOffset %u, uiLength %u",
				__FUNCTION__,
				uiOffset,
				uiLength));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	sPhysStart.uiAddr = page_to_phys(psPage) + uiOffset;
	sPhysEnd.uiAddr = sPhysStart.uiAddr + uiLength;

	OSCleanCPUCacheRangeKM(pvVirtAddrStart,
	                       pvVirtAddrStart + uiLength,
	                       sPhysStart,
	                       sPhysEnd);

e0:
	kunmap(psPage);

	return eError;
}

#if defined(__GNUC__)
#define PVRSRV_MEM_ALIGN __attribute__ ((aligned (0x8)))
#define PVRSRV_MEM_ALIGN_MASK (0x7)
#else
#error "PVRSRV Alignment macros need to be defined for this compiler"
#endif

/*************************************************************************/ /*!
@Function       OSCPUCacheAttributeSize
@Description    Lookup dcache attribute sizes
@Input          eCacheAttribute
*/ /**************************************************************************/
IMG_UINT32 OSCPUCacheAttributeSize(IMG_DCACHE_ATTRIBUTE eCacheAttribute)
{
	IMG_UINT32 uiSize = 0;

	switch(eCacheAttribute)
	{
		case PVR_DCACHE_LINE_SIZE:
			uiSize = cache_line_size();
			break;

		default:
			PVR_DPF((PVR_DBG_ERROR, "%s: Invalid cache attribute type %d",
					__FUNCTION__, (IMG_UINT32)eCacheAttribute));
			PVR_ASSERT(0);
			break;
	}

	return uiSize;
}

IMG_INT OSMemCmp(void *pvBufA, void *pvBufB, size_t uiLen)
{
	return (IMG_INT) memcmp(pvBufA, pvBufB, uiLen);
}

/*************************************************************************/ /*!
@Function       OSStringNCopy
@Description    strcpy
*/ /**************************************************************************/
IMG_CHAR *OSStringNCopy(IMG_CHAR *pszDest, const IMG_CHAR *pszSrc, size_t uSize)
{
	return strncpy(pszDest, pszSrc, uSize);
}

/*************************************************************************/ /*!
@Function       OSSNPrintf
@Description    snprintf
@Return         the chars written or -1 on error
*/ /**************************************************************************/
IMG_INT32 OSSNPrintf(IMG_CHAR *pStr, size_t ui32Size, const IMG_CHAR *pszFormat, ...)
{
	va_list argList;
	IMG_INT32 iCount;

	va_start(argList, pszFormat);
	iCount = vsnprintf(pStr, (size_t)ui32Size, pszFormat, argList);
	va_end(argList);

	return iCount;
}

size_t OSStringLength(const IMG_CHAR *pStr)
{
	return strlen(pStr);
}

size_t OSStringNLength(const IMG_CHAR *pStr, size_t uiCount)
{
	return strnlen(pStr, uiCount);
}

IMG_INT32 OSStringCompare(const IMG_CHAR *pStr1, const IMG_CHAR *pStr2)
{
	return strcmp(pStr1, pStr2);
}

/*************************************************************************/ /*!
@Function       OSInitEnvData
@Description    Allocates space for env specific data
@Input          ppvEnvSpecificData   Pointer to pointer in which to return
                                     allocated data.
@Input          ui32MMUMode          MMU mode.
@Return         PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR OSInitEnvData(void)
{
	/* allocate memory for the bridge buffers to be used during an ioctl */
	g_pvBridgeBuffers = OSAllocMem(PVRSRV_MAX_BRIDGE_IN_SIZE + PVRSRV_MAX_BRIDGE_OUT_SIZE);
	if (g_pvBridgeBuffers == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	atomic_set(&g_DriverSuspended, 0);

	LinuxInitPhysmem();

	return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSDeInitEnvData
@Description    frees env specific data memory
@Input          pvEnvSpecificData   Pointer to private structure
@Return         PVRSRV_OK on success else PVRSRV_ERROR_OUT_OF_MEMORY
*/ /**************************************************************************/
void OSDeInitEnvData(void)
{

	LinuxDeinitPhysmem();

	if (g_pvBridgeBuffers)
	{
		/* free-up the memory allocated for bridge buffers */
		OSFreeMem(g_pvBridgeBuffers);
		g_pvBridgeBuffers = NULL;
	}
}

PVRSRV_ERROR OSGetGlobalBridgeBuffers(void **ppvBridgeInBuffer,
							IMG_UINT32 *pui32BridgeInBufferSize,
							void **ppvBridgeOutBuffer,
							IMG_UINT32 *pui32BridgeOutBufferSize)
{
	PVR_ASSERT (ppvBridgeInBuffer && ppvBridgeOutBuffer);
	PVR_ASSERT (pui32BridgeInBufferSize && pui32BridgeOutBufferSize);

	*ppvBridgeInBuffer = g_pvBridgeBuffers;
	*pui32BridgeInBufferSize = PVRSRV_MAX_BRIDGE_IN_SIZE;

	*ppvBridgeOutBuffer = *ppvBridgeInBuffer + *pui32BridgeInBufferSize;
	*pui32BridgeOutBufferSize = PVRSRV_MAX_BRIDGE_OUT_SIZE;

	return PVRSRV_OK;
}

IMG_BOOL OSSetDriverSuspended(void)
{
	int suspend_level = atomic_inc_return(&g_DriverSuspended);
	return (1 != suspend_level)? IMG_FALSE: IMG_TRUE;
}

IMG_BOOL OSClearDriverSuspended(void)
{
	int suspend_level = atomic_dec_return(&g_DriverSuspended);
	return (0 != suspend_level)? IMG_FALSE: IMG_TRUE;
}

IMG_BOOL OSGetDriverSuspended(void)
{
	return (0 < atomic_read(&g_DriverSuspended))? IMG_TRUE: IMG_FALSE;
}

/*************************************************************************/ /*!
@Function       OSReleaseThreadQuanta
@Description    Releases thread quanta
*/ /**************************************************************************/ 
void OSReleaseThreadQuanta(void)
{
	schedule();
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static inline IMG_UINT32 Clockus(void)
{
	return (jiffies * (1000000 / HZ));
}
#else
/* Not matching/aligning this API to the Clockus() API above to avoid necessary
 * multiplication/division operations in calling code.
 */
static inline IMG_UINT64 Clockns64(void)
{
	IMG_UINT64 timenow;

	/* Kernel thread preempt protection. Some architecture implementations 
	 * (ARM) of sched_clock are not preempt safe when the kernel is configured 
	 * as such e.g. CONFIG_PREEMPT and others.
	 */
	preempt_disable();

	/* Using sched_clock instead of ktime_get since we need a time stamp that
	 * correlates with that shown in kernel logs and trace data not one that
	 * is a bit behind. */
	timenow = sched_clock();

	preempt_enable();

	return timenow;
}
#endif

/*************************************************************************/ /*!
 @Function OSClockns64
 @Description
        This function returns the clock in nanoseconds. Unlike OSClockus,
        OSClockus64 has a near 64-bit range
*/ /**************************************************************************/
IMG_UINT64 OSClockns64(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	return Clockns64();	
#else
	return ((IMG_UINT64)Clockus()) * 1000ULL;
#endif
}

/*************************************************************************/ /*!
 @Function OSClockus64
 @Description
        This function returns the clock in microseconds. Unlike OSClockus,
        OSClockus64 has a near 64-bit range
*/ /**************************************************************************/
IMG_UINT64 OSClockus64(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	IMG_UINT64 timenow = Clockns64();
	IMG_UINT32 remainder;
	return OSDivide64r64(timenow, 1000, &remainder);
#else
	return ((IMG_UINT64)Clockus());
#endif
}


/*************************************************************************/ /*!
@Function       OSClockus
@Description    This function returns the clock in microseconds
@Return         clock (us)
*/ /**************************************************************************/ 
IMG_UINT32 OSClockus(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	return (IMG_UINT32) OSClockus64();
#else
	return Clockus();
#endif
}


/*************************************************************************/ /*!
@Function       OSClockms
@Description    This function returns the clock in milliseconds
@Return         clock (ms)
*/ /**************************************************************************/ 
IMG_UINT32 OSClockms(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	IMG_UINT64 timenow = Clockns64();
	IMG_UINT32 remainder;

	return OSDivide64(timenow, 1000000, &remainder);
#else
	IMG_UINT64 time, j = (IMG_UINT32)jiffies;

	time = j * (((1 << 16) * 1000) / HZ);
	time >>= 16;

	return (IMG_UINT32)time;
#endif
}


/*
	OSWaitus
*/
void OSWaitus(IMG_UINT32 ui32Timeus)
{
	udelay(ui32Timeus);
}


/*
	OSSleepms
*/
void OSSleepms(IMG_UINT32 ui32Timems)
{
	msleep(ui32Timems);
}


/*************************************************************************/ /*!
@Function       OSGetCurrentProcessID
@Description    Returns ID of current process (thread group)
@Return         ID of current process
*****************************************************************************/
INLINE IMG_PID OSGetCurrentProcessID(void)
{
	if (in_interrupt())
	{
		return KERNEL_ID;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	return (IMG_PID)current->pgrp;
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24))
	return (IMG_PID)task_tgid_nr(current);
#else
	return (IMG_PID)current->tgid;
#endif
#endif
}

/*************************************************************************/ /*!
@Function       OSGetCurrentProcessName
@Description    gets name of current process
@Return         process name
*****************************************************************************/
INLINE IMG_CHAR *OSGetCurrentProcessName(void)
{
	return current->comm;
}

/*************************************************************************/ /*!
@Function       OSGetCurrentThreadID
@Description    Returns ID for current thread
@Return         ID of current thread
*****************************************************************************/
INLINE uintptr_t OSGetCurrentThreadID(void)
{
	if (in_interrupt())
	{
		return KERNEL_ID;
	}

	return current->pid;
}

/*************************************************************************/ /*!
@Function       OSGetCurrentClientProcessIDKM
@Description    Returns ID of current client process (thread group)
@Return         ID of current client process
*****************************************************************************/
IMG_PID OSGetCurrentClientProcessIDKM(void)
{
	return OSGetCurrentProcessID();
}

/*************************************************************************/ /*!
@Function       OSGetCurrentClientProcessNameKM
@Description    gets name of current client process
@Return         client process name
*****************************************************************************/
IMG_CHAR *OSGetCurrentClientProcessNameKM(void)
{
	return OSGetCurrentProcessName();
}

/*************************************************************************/ /*!
@Function       OSGetCurrentClientThreadIDKM
@Description    Returns ID for current client thread
@Return         ID of current client thread
*****************************************************************************/
uintptr_t OSGetCurrentClientThreadIDKM(void)
{
	return OSGetCurrentThreadID();
}
/*************************************************************************/ /*!
@Function       OSGetPageSize
@Description    gets page size
@Return         page size
*/ /**************************************************************************/
size_t OSGetPageSize(void)
{
	return PAGE_SIZE;
}

/*************************************************************************/ /*!
@Function       OSGetPageShift
@Description    gets page size
@Return         page size
*/ /**************************************************************************/
size_t OSGetPageShift(void)
{
	return PAGE_SHIFT;
}

/*************************************************************************/ /*!
@Function       OSGetPageMask
@Description    gets page mask
@Return         page size
*/ /**************************************************************************/
size_t OSGetPageMask(void)
{
	return (OSGetPageSize()-1);
}

/*************************************************************************/ /*!
@Function       OSGetOrder
@Description    gets log base 2 (Order) value of the given size
@Return         order
*/ /**************************************************************************/
size_t OSGetOrder(size_t uSize)
{
	return get_order(PAGE_ALIGN(uSize));
}

#if !defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
#if defined(PVRSRV_GPUVIRT_GUESTDRV) && defined(PVRSRV_GPUVIRT_MULTIDRV_MODEL)
/*
	Device interrupt (ISR/LISR) management is predicated on the following:
		- For normal/hyperv drivers:
			- Perform device interrupt management directly (normal case)
		- For guest drivers, behaviour depends on:
			- If running on a multi-driver model (same OS instance)
				- Delegate management to hypervisor driver
				- Register guest driver device LISRs with hyperv
				- Hypervisor triggers guests driver device LISRs
			- Else assume hypervisor vm monitor exposes device/irq abstraction
				- Manage this virtual device/irq directly like a normal driver
				- Hypervisor vm monitor triggers device/irq abstraction
				- Setup for this is outside the scope of the DDK
 */
#else
typedef struct _LISR_DATA_ {
	PFN_LISR pfnLISR;
	void *pvData;
	IMG_UINT32 ui32IRQ;
} LISR_DATA;

/*
	DeviceISRWrapper
*/
static irqreturn_t DeviceISRWrapper(int irq, void *dev_id)
{
	LISR_DATA *psLISRData = (LISR_DATA *) dev_id;
	IMG_BOOL bStatus = IMG_FALSE;

	PVR_UNREFERENCED_PARAMETER(irq);

#if defined(SUPPORT_PVRSRV_GPUVIRT) && defined(PVRSRV_GPUVIRT_MULTIDRV_MODEL)
	bStatus = SysVirtTriggerAllGuestDeviceLISR() == PVRSRV_OK ? IMG_TRUE : IMG_FALSE;
#endif
	bStatus |= psLISRData->pfnLISR(psLISRData->pvData);

	return bStatus ? IRQ_HANDLED : IRQ_NONE;
}
#endif
#endif

/*
	OSInstallDeviceLISR
*/
PVRSRV_ERROR OSInstallDeviceLISR(PVRSRV_DEVICE_CONFIG *psDevConfig,
				 IMG_HANDLE *hLISRData, 
				 PFN_LISR pfnLISR,
				 void *pvData)
{
#if defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
	return SysInstallDeviceLISR(psDevConfig->ui32IRQ,
					psDevConfig->pszName,
					pfnLISR,
					pvData,
					hLISRData);
#else
#if defined(PVRSRV_GPUVIRT_GUESTDRV) && defined(PVRSRV_GPUVIRT_MULTIDRV_MODEL)
	return GuestBridgeSysInstallDeviceLISR(PVRSRV_GPUVIRT_OSID,
									psDevConfig->ui32IRQ,
									psDevConfig->pszName,
									pfnLISR,
									pvData,
									hLISRData);
#else
	LISR_DATA *psLISRData;
	unsigned long flags = 0;

	psLISRData = OSAllocMem(sizeof(LISR_DATA));

	psLISRData->pfnLISR = pfnLISR;
	psLISRData->pvData = pvData;
	psLISRData->ui32IRQ = psDevConfig->ui32IRQ;

	if (psDevConfig->bIRQIsShared)
	{
		flags |= IRQF_SHARED;
	}

	if (psDevConfig->eIRQActiveLevel == PVRSRV_DEVICE_IRQ_ACTIVE_HIGH)
	{
		flags |= IRQF_TRIGGER_HIGH;
	}
	else if (psDevConfig->eIRQActiveLevel == PVRSRV_DEVICE_IRQ_ACTIVE_LOW)
	{
		flags |= IRQF_TRIGGER_LOW;
	}

	PVR_TRACE(("Installing device LISR %s on IRQ %d with cookie %p", 
				psDevConfig->pszName, psDevConfig->ui32IRQ, pvData));

	if(request_irq(psDevConfig->ui32IRQ, DeviceISRWrapper,
		flags, psDevConfig->pszName, psLISRData))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"OSInstallDeviceLISR: Couldn't install device LISR on IRQ %d", 
				psDevConfig->ui32IRQ));

		return PVRSRV_ERROR_UNABLE_TO_INSTALL_ISR;
	}

	*hLISRData = (IMG_HANDLE) psLISRData;

	return PVRSRV_OK;
#endif
#endif
}

/*
	OSUninstallDeviceLISR
*/
PVRSRV_ERROR OSUninstallDeviceLISR(IMG_HANDLE hLISRData)
{
#if defined (SUPPORT_SYSTEM_INTERRUPT_HANDLING)
	return SysUninstallDeviceLISR(hLISRData);
#else
#if defined(PVRSRV_GPUVIRT_GUESTDRV) && defined(PVRSRV_GPUVIRT_MULTIDRV_MODEL)
	return GuestBridgeSysUninstallDeviceLISR(PVRSRV_GPUVIRT_OSID, hLISRData);
#else
	LISR_DATA *psLISRData = (LISR_DATA *) hLISRData;

	PVR_TRACE(("Uninstalling device LISR on IRQ %d with cookie %p", psLISRData->ui32IRQ,  psLISRData->pvData));

	free_irq(psLISRData->ui32IRQ, psLISRData);
	OSFreeMem(psLISRData);

	return PVRSRV_OK;
#endif	
#endif
}

#if defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct workqueue_struct *psWorkQueue;
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR,
							void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = OSAllocMem(sizeof(MISR_DATA));
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	psMISRData->psWorkQueue = create_singlethread_workqueue("pvr_workqueue" PVRSRV_GPUVIRT_OSID_STR);

	if (psMISRData->psWorkQueue == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSInstallMISR: create_singlethreaded_workqueue failed"));
		OSFreeMem(psMISRData);
		return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;
	}

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}

/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	PVR_TRACE(("Uninstalling MISR"));

	destroy_workqueue(psMISRData->psWorkQueue);
	OSFreeMem(psMISRData);

	return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	/*
		Note:

		In the case of NO_HARDWARE we want the driver to be synchronous so
		that we don't have to worry about waiting for previous operations
		to complete
	*/
#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	queue_work(psMISRData->psWorkQueue, &psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}
#else	/* defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) */
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = OSAllocMem(sizeof(MISR_DATA));
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}


/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	PVR_TRACE(("Uninstalling MISR"));

	flush_scheduled_work();

	OSFreeMem(hMISRData);

	return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = hMISRData;
#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	schedule_work(&psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}

#else	/* #if defined(PVR_LINUX_MISR_USING_WORKQUEUE) */
typedef struct _MISR_DATA_ {
	struct tasklet_struct sMISRTasklet;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(unsigned long data)
{
	MISR_DATA *psMISRData = (MISR_DATA *) data;

	psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = OSAllocMem(sizeof(MISR_DATA));
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	tasklet_init(&psMISRData->sMISRTasklet, MISRWrapper, (unsigned long)psMISRData);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}

/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	PVR_TRACE(("Uninstalling MISR"));

	tasklet_kill(&psMISRData->sMISRTasklet);

	return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	tasklet_schedule(&psMISRData->sMISRTasklet);
#endif
	return PVRSRV_OK;
}

#endif /* #if defined(PVR_LINUX_MISR_USING_WORKQUEUE) */
#endif /* #if defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) */

/* OS specific values for thread priority */
const IMG_INT32 ai32OSPriorityValues[OS_THREAD_LAST_PRIORITY] = { -20, /* OS_THREAD_HIGHEST_PRIORITY */
																  -10, /* OS_THREAD_HIGH_PRIORITY */
																	0, /* OS_THREAD_NORMAL_PRIORITY */
																	9, /* OS_THREAD_LOW_PRIORITY */
																   19, /* OS_THREAD_LOWEST_PRIORITY */
																  -22};/* OS_THREAD_NOSET_PRIORITY */

typedef struct {
	struct task_struct *kthread;
	PFN_THREAD pfnThread;
	void *hData;
	OS_THREAD_LEVEL eThreadPriority;
} OSThreadData;

static int OSThreadRun(void *data)
{
	OSThreadData *psOSThreadData = data;

	/* If i32NiceValue is acceptable, set the nice value for the new thread */
	if (psOSThreadData->eThreadPriority != OS_THREAD_NOSET_PRIORITY &&
	         psOSThreadData->eThreadPriority < OS_THREAD_LAST_PRIORITY)
		set_user_nice(current, ai32OSPriorityValues[psOSThreadData->eThreadPriority]);

	/* Call the client's kernel thread with the client's data pointer */
	psOSThreadData->pfnThread(psOSThreadData->hData);

	/* Wait for OSThreadDestroy() to call kthread_stop() */
	while (!kthread_should_stop())
	{
		 schedule();
	}

	return 0;
}

PVRSRV_ERROR OSThreadCreate(IMG_HANDLE *phThread,
							IMG_CHAR *pszThreadName,
							PFN_THREAD pfnThread,
							void *hData)
{
	return OSThreadCreatePriority(phThread, pszThreadName, pfnThread, hData, OS_THREAD_NOSET_PRIORITY);
}

PVRSRV_ERROR OSThreadCreatePriority(IMG_HANDLE *phThread,
									IMG_CHAR *pszThreadName,
									PFN_THREAD pfnThread,
									void *hData,
									OS_THREAD_LEVEL eThreadPriority)
{
	OSThreadData *psOSThreadData;
	PVRSRV_ERROR eError;

	psOSThreadData = OSAllocMem(sizeof(OSThreadData));
	if (psOSThreadData == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	psOSThreadData->pfnThread = pfnThread;
	psOSThreadData->hData = hData;
	psOSThreadData->eThreadPriority= eThreadPriority;
	psOSThreadData->kthread = kthread_run(OSThreadRun, psOSThreadData, pszThreadName);

	if (IS_ERR(psOSThreadData->kthread))
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_kthread;
	}

	*phThread = psOSThreadData;

	return PVRSRV_OK;

fail_kthread:
	OSFreeMem(psOSThreadData);
fail_alloc:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR OSThreadDestroy(IMG_HANDLE hThread)
{
	OSThreadData *psOSThreadData = hThread;
	int ret;

	/* Let the thread know we are ready for it to end and wait for it. */
	ret = kthread_stop(psOSThreadData->kthread);
	PVR_ASSERT(ret == 0);
	OSFreeMem(psOSThreadData);

	return PVRSRV_OK;
}

void OSPanic(void)
{
	BUG();

#if defined(__KLOCWORK__)
	/* Klocworks does not understand that BUG is terminal... */
	abort();
#endif
}

/*************************************************************************/ /*!
@Function       OSMapPhysToLin
@Description    Maps the physical memory into linear addr range
@Input          BasePAddr       Physical cpu address
@Input          ui32Bytes       Bytes to map
@Input          ui32CacheType   Cache type
@Return         Linear addr of mapping on success, else NULL
 */ /**************************************************************************/
void *
OSMapPhysToLin(IMG_CPU_PHYADDR BasePAddr,
			   size_t ui32Bytes,
			   IMG_UINT32 ui32MappingFlags)
{
	void *pvIORemapCookie;

	pvIORemapCookie = IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags);
	if(pvIORemapCookie == NULL)
	{
		PVR_ASSERT(0);
		return NULL;
	}

	return pvIORemapCookie;
}


/*************************************************************************/ /*!
@Function       OSUnMapPhysToLin
@Description    Unmaps memory that was mapped with OSMapPhysToLin
@Input          pvLinAddr
@Input          ui32Bytes
@Return         TRUE on success, else FALSE
*/ /**************************************************************************/
IMG_BOOL
OSUnMapPhysToLin(void *pvLinAddr, size_t ui32Bytes, IMG_UINT32 ui32MappingFlags)
{
	PVR_UNREFERENCED_PARAMETER(ui32Bytes);

	IOUnmapWrapper(pvLinAddr);

	return IMG_TRUE;
}

/*
	OSReadHWReg8
*/
IMG_UINT8 OSReadHWReg8(void *pvLinRegBaseAddr,
						IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT8) readb((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x4e;
#endif
}

/*
	OSReadHWReg16
*/
IMG_UINT16 OSReadHWReg16(void *pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT16) readw((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x3a4e;
#endif
}

/*
	OSReadHWReg32
*/
IMG_UINT32 OSReadHWReg32(void *pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT32) readl((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x30f73a4e;
#endif
}


/*
	OSReadHWReg64
*/
IMG_UINT64 OSReadHWReg64(void *pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
	IMG_UINT64	ui64Result;

	ui64Result = OSReadHWReg32(pvLinRegBaseAddr, ui32Offset + 4);
	ui64Result <<= 32;
	ui64Result |= (IMG_UINT64)OSReadHWReg32(pvLinRegBaseAddr, ui32Offset);

	return ui64Result;
}

/*
	OSReadHWRegBank
*/
IMG_DEVMEM_SIZE_T OSReadHWRegBank(void *pvLinRegBaseAddr,
								  IMG_UINT32 ui32Offset,
								  IMG_UINT8 *pui8DstBuf,
								  IMG_DEVMEM_SIZE_T uiDstBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;

	for(uiCounter = 0; uiCounter < uiDstBufLen; uiCounter++) {
		*(pui8DstBuf + uiCounter) =
		  readb(pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiDstBufLen;
#endif
}

/*
	OSWriteHWReg8
*/
void OSWriteHWReg8(void			*pvLinRegBaseAddr,
				   IMG_UINT32	ui32Offset,
				   IMG_UINT8	ui8Value)
{
#if !defined(NO_HARDWARE)
	writeb(ui8Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

/*
	OSWriteHWReg16
*/
void OSWriteHWReg16(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT16	ui16Value)
{
#if !defined(NO_HARDWARE)
	writew(ui16Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

/*
	OSWriteHWReg32
*/
void OSWriteHWReg32(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT32	ui32Value)
{
#if !defined(NO_HARDWARE)
	writel(ui32Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}


/*
	OSWriteHWReg64
*/
void OSWriteHWReg64(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT64	ui64Value)
{
#if !defined(NO_HARDWARE)
	IMG_UINT32 ui32ValueLow, ui32ValueHigh;

	ui32ValueLow = ui64Value & 0xffffffff;
	ui32ValueHigh = ((IMG_UINT64) (ui64Value >> 32)) & 0xffffffff;

	writel(ui32ValueLow, pvLinRegBaseAddr + ui32Offset);
	writel(ui32ValueHigh, pvLinRegBaseAddr + ui32Offset + 4);
#endif
}

IMG_DEVMEM_SIZE_T OSWriteHWRegBank(void *pvLinRegBaseAddr,
								   IMG_UINT32 ui32Offset,
								   IMG_UINT8 *pui8SrcBuf,
								   IMG_DEVMEM_SIZE_T uiSrcBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;

	for(uiCounter = 0; uiCounter < uiSrcBufLen; uiCounter++) {
		writeb(*(pui8SrcBuf + uiCounter),
		       pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiSrcBufLen;
#endif
}

#define	OS_MAX_TIMERS	8

/* Timer callback strucure used by OSAddTimer */
typedef struct TIMER_CALLBACK_DATA_TAG
{
	IMG_BOOL			bInUse;
	PFN_TIMER_FUNC		pfnTimerFunc;
	void				*pvData;
	struct timer_list	sTimer;
	IMG_UINT32			ui32Delay;
	IMG_BOOL			bActive;
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	struct work_struct	sWork;
#endif
}TIMER_CALLBACK_DATA;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
static struct workqueue_struct	*psTimerWorkQueue;
#endif

static TIMER_CALLBACK_DATA sTimers[OS_MAX_TIMERS];

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
DEFINE_MUTEX(sTimerStructLock);
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
/* The lock is used to control access to sTimers */
static spinlock_t sTimerStructLock = SPIN_LOCK_UNLOCKED;
#else
static DEFINE_SPINLOCK(sTimerStructLock);
#endif
#endif

static void OSTimerCallbackBody(TIMER_CALLBACK_DATA *psTimerCBData)
{
	if (!psTimerCBData->bActive)
		return;

	/* call timer callback */
	psTimerCBData->pfnTimerFunc(psTimerCBData->pvData);

	/* reset timer */
	mod_timer(&psTimerCBData->sTimer, psTimerCBData->ui32Delay + jiffies);
}


/*************************************************************************/ /*!
@Function       OSTimerCallbackWrapper
@Description    OS specific timer callback wrapper function
@Input          uData    Timer callback data
*/ /**************************************************************************/
static void OSTimerCallbackWrapper(uintptr_t uData)
{
	TIMER_CALLBACK_DATA	*psTimerCBData = (TIMER_CALLBACK_DATA*)uData;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	int res;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	res = queue_work(psTimerWorkQueue, &psTimerCBData->sWork);
#else
	res = schedule_work(&psTimerCBData->sWork);
#endif
	if (res == 0)
	{
		PVR_DPF((PVR_DBG_WARNING, "OSTimerCallbackWrapper: work already queued"));
	}
#else
	OSTimerCallbackBody(psTimerCBData);
#endif
}


#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
static void OSTimerWorkQueueCallBack(struct work_struct *psWork)
{
	TIMER_CALLBACK_DATA *psTimerCBData = container_of(psWork, TIMER_CALLBACK_DATA, sWork);

	OSTimerCallbackBody(psTimerCBData);
}
#endif

/*************************************************************************/ /*!
@Function       OSAddTimer
@Description    OS specific function to install a timer callback
@Input          pfnTimerFunc    Timer callback
@Input         *pvData          Callback data
@Input          ui32MsTimeout   Callback period
@Return         Valid handle success, NULL failure
*/ /**************************************************************************/
IMG_HANDLE OSAddTimer(PFN_TIMER_FUNC pfnTimerFunc, void *pvData, IMG_UINT32 ui32MsTimeout)
{
	TIMER_CALLBACK_DATA	*psTimerCBData;
	IMG_UINT32		ui32i;
#if !(defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE))
	unsigned long		ulLockFlags;
#endif

	/* check callback */
	if(!pfnTimerFunc)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: passed invalid callback"));
		return NULL;
	}

	/* Allocate timer callback data structure */
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	mutex_lock(&sTimerStructLock);
#else
	spin_lock_irqsave(&sTimerStructLock, ulLockFlags);
#endif
	for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
	{
		psTimerCBData = &sTimers[ui32i];
		if (!psTimerCBData->bInUse)
		{
			psTimerCBData->bInUse = IMG_TRUE;
			break;
		}
	}
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	mutex_unlock(&sTimerStructLock);
#else
	spin_unlock_irqrestore(&sTimerStructLock, ulLockFlags);
#endif
	if (ui32i >= OS_MAX_TIMERS)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: all timers are in use"));
		return NULL;
	}

	psTimerCBData->pfnTimerFunc = pfnTimerFunc;
	psTimerCBData->pvData = pvData;
	psTimerCBData->bActive = IMG_FALSE;

	/*
		HZ = ticks per second
		ui32MsTimeout = required ms delay
		ticks = (Hz * ui32MsTimeout) / 1000
	*/
	psTimerCBData->ui32Delay = ((HZ * ui32MsTimeout) < 1000)
								?	1
								:	((HZ * ui32MsTimeout) / 1000);
	/* initialise object */
	init_timer(&psTimerCBData->sTimer);

	/* setup timer object */
	psTimerCBData->sTimer.function = (void *)OSTimerCallbackWrapper;
	psTimerCBData->sTimer.data = (uintptr_t)psTimerCBData;

	return (IMG_HANDLE)(uintptr_t)(ui32i + 1);
}


static inline TIMER_CALLBACK_DATA *GetTimerStructure(IMG_HANDLE hTimer)
{
	IMG_UINT32 ui32i = (IMG_UINT32)((uintptr_t)hTimer) - 1;

	PVR_ASSERT(ui32i < OS_MAX_TIMERS);

	return &sTimers[ui32i];
}

/*************************************************************************/ /*!
@Function       OSRemoveTimer
@Description    OS specific function to remove a timer callback
@Input          hTimer : timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSRemoveTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(!psTimerCBData->bActive);

	/* free timer callback data struct */
	psTimerCBData->bInUse = IMG_FALSE;

	return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSEnableTimer
@Description    OS specific function to enable a timer callback
@Input          hTimer    Timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEnableTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(!psTimerCBData->bActive);

	/* Start timer arming */
	psTimerCBData->bActive = IMG_TRUE;

	/* set the expire time */
	psTimerCBData->sTimer.expires = psTimerCBData->ui32Delay + jiffies;

	/* Add the timer to the list */
	add_timer(&psTimerCBData->sTimer);

	return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSDisableTimer
@Description    OS specific function to disable a timer callback
@Input          hTimer    Timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSDisableTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(psTimerCBData->bActive);

	/* Stop timer from arming */
	psTimerCBData->bActive = IMG_FALSE;
	smp_mb();

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	flush_scheduled_work();
#endif

	/* remove timer */
	del_timer_sync(&psTimerCBData->sTimer);

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	/*
	 * This second flush is to catch the case where the timer ran
	 * before we managed to delete it, in which case, it will have
	 * queued more work for the workqueue.	Since the bActive flag
	 * has been cleared, this second flush won't result in the
	 * timer being rearmed.
	 */
	flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	flush_scheduled_work();
#endif

	return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSEventObjectCreate
@Description    OS specific function to create an event object
@Input          pszName      Globally unique event object name (if null name must be autogenerated)
@Output         hEventObject OS event object info structure
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectCreate(const IMG_CHAR *pszName, IMG_HANDLE *hEventObject)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVR_UNREFERENCED_PARAMETER(pszName);

	if(hEventObject)
	{
		if(LinuxEventObjectListCreate(hEventObject) != PVRSRV_OK)
		{
			 eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		}

	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectCreate: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_UNABLE_TO_CREATE_EVENT;
	}

	return eError;
}


/*************************************************************************/ /*!
@Function       OSEventObjectDestroy
@Description    OS specific function to destroy an event object
@Input          hEventObject   OS event object info structure
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectDestroy(IMG_HANDLE hEventObject)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hEventObject)
	{
		LinuxEventObjectListDestroy(hEventObject);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

/*
 * EventObjectWaitTimeout()
 */
static PVRSRV_ERROR EventObjectWaitTimeout(IMG_HANDLE hOSEventKM,
										   IMG_UINT32 uiTimeoutMs,
										   IMG_BOOL bHoldBridgeLock)
{
    PVRSRV_ERROR eError;

	if(hOSEventKM && uiTimeoutMs > 0)
	{
		eError = LinuxEventObjectWait(hOSEventKM, uiTimeoutMs, bHoldBridgeLock);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectWait: invalid arguments %p, %d", hOSEventKM, uiTimeoutMs ));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectWaitTimeout
@Description    Wait for an event with timeout as supplied. Called from client
@Input          hOSEventKM    OS and kernel specific handle to event object
@Input          uiTimeoutMs   Non zero time period in milliseconds to wait
@Return         PVRSRV_ERROR_TIMEOUT : Wait reached wait limit and timed out
@Return         PVRSRV_ERROR         : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWaitTimeout(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs)
{
    return EventObjectWaitTimeout(hOSEventKM, uiTimeoutMs, IMG_FALSE);
}

/*************************************************************************/ /*!
@Function       OSEventObjectWait
@Description    OS specific function to wait for an event object. Called
				from client. Uses a default wait with 100ms timeout.
@Input          hOSEventKM    OS and kernel specific handle to event object
@Return         PVRSRV_ERROR_TIMEOUT  : Reached wait limit and timed out
@Return         PVRSRV_ERROR  : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWait(IMG_HANDLE hOSEventKM)
{
	return OSEventObjectWaitTimeout(hOSEventKM, EVENT_OBJECT_TIMEOUT_MS);
}

/*************************************************************************/ /*!
@Function       OSEventObjectWaitTimeoutAndHoldBridgeLock
@Description    Wait for an event with timeout as supplied. Called from client
                NOTE: Holds bridge lock during wait.
@Input          hOSEventKM    OS and kernel specific handle to event object
@Input          uiTimeoutMs   Non zero time period in milliseconds to wait
@Return         PVRSRV_ERROR_TIMEOUT : Wait reached wait limit and timed out
@Return         PVRSRV_ERROR         : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWaitTimeoutAndHoldBridgeLock(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs)
{
	return EventObjectWaitTimeout(hOSEventKM, uiTimeoutMs, IMG_TRUE);
}

/*************************************************************************/ /*!
@Function       OSEventObjectWaitAndHoldBridgeLock
@Description    OS specific function to wait for an event object. Called
				from client. Uses a default wait with 100ms timeout.
                NOTE: Holds bridge lock during wait.
@Input          hOSEventKM    OS and kernel specific handle to event object
@Return         PVRSRV_ERROR_TIMEOUT  : Reached wait limit and timed out
@Return         PVRSRV_ERROR  : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWaitAndHoldBridgeLock(IMG_HANDLE hOSEventKM)
{
	return OSEventObjectWaitTimeoutAndHoldBridgeLock(hOSEventKM, EVENT_OBJECT_TIMEOUT_MS);
}

/*************************************************************************/ /*!
@Function       OSEventObjectOpen
@Description    OS specific function to open an event object.  Called from client
@Input          hEventObject  Pointer to an event object
@Output         phOSEvent     OS and kernel specific handle to event object
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectOpen(IMG_HANDLE hEventObject,
											IMG_HANDLE *phOSEvent)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hEventObject)
	{
		if(LinuxEventObjectAdd(hEventObject, phOSEvent) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectAdd: failed"));
			eError = PVRSRV_ERROR_INVALID_PARAMS;
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectOpen: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectClose
@Description    OS specific function to close an event object.  Called from client
@Input          hOSEventKM    OS and kernel specific handle to event object
@Return         PVRSRV_ERROR  :
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectClose(IMG_HANDLE hOSEventKM)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hOSEventKM)
	{
		if(LinuxEventObjectDelete(hOSEventKM) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectDelete: failed"));
			eError = PVRSRV_ERROR_INVALID_PARAMS;
		}

	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectSignal
@Description    OS specific function to 'signal' an event object.  Called from L/MISR
@Input          hOSEventKM   OS and kernel specific handle to event object
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectSignal(IMG_HANDLE hEventObject)
{
	PVRSRV_ERROR eError;

	if(hEventObject)
	{
		eError = LinuxEventObjectSignal(hEventObject);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectSignal: hOSEventKM is not a valid handle"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

/*************************************************************************/ /*!
@Function       OSProcHasPrivSrvInit
@Description    Does the process have sufficient privileges to initialise services?
@Return         IMG_BOOL
*/ /**************************************************************************/
IMG_BOOL OSProcHasPrivSrvInit(void)
{
	return capable(CAP_SYS_ADMIN) != 0;
}

/*************************************************************************/ /*!
@Function       OSCopyToUser
@Description    Copy a block of data into user space
@Input          pvSrc
@Output         pvDest
@Input          ui32Bytes
@Return   PVRSRV_ERROR  :
*/ /**************************************************************************/
PVRSRV_ERROR OSCopyToUser(void *pvProcess,
						  void *pvDest,
						  const void *pvSrc,
						  size_t ui32Bytes)
{
	PVR_UNREFERENCED_PARAMETER(pvProcess);

	if(pvr_copy_to_user(pvDest, pvSrc, ui32Bytes)==0)
		return PVRSRV_OK;
	else
		return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

/*************************************************************************/ /*!
@Function       OSCopyFromUser
@Description    Copy a block of data from the user space
@Output         pvDest
@Input          pvSrc
@Input          ui32Bytes
@Return         PVRSRV_ERROR  :
*/ /**************************************************************************/
PVRSRV_ERROR OSCopyFromUser(void *pvProcess,
							void *pvDest,
							const void *pvSrc,
							size_t ui32Bytes)
{
	PVR_UNREFERENCED_PARAMETER(pvProcess);

	if(pvr_copy_from_user(pvDest, pvSrc, ui32Bytes)==0)
		return PVRSRV_OK;
	else
		return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

/*************************************************************************/ /*!
@Function       OSAccessOK
@Description    Checks if a user space pointer is valide
@Input          eVerification
@Input          pvUserPtr
@Input          ui32Bytes
@Return         IMG_BOOL :
*/ /**************************************************************************/
IMG_BOOL OSAccessOK(IMG_VERIFY_TEST eVerification, void *pvUserPtr, size_t ui32Bytes)
{
	IMG_INT linuxType;

	if (eVerification == PVR_VERIFY_READ)
	{
		linuxType = VERIFY_READ;
	}
	else
	{
		PVR_ASSERT(eVerification == PVR_VERIFY_WRITE);
		linuxType = VERIFY_WRITE;
	}

	return access_ok(linuxType, pvUserPtr, ui32Bytes);
}


void OSWriteMemoryBarrier(void)
{
	wmb();
}


void OSMemoryBarrier(void)
{
	mb();
}

IMG_UINT64 OSDivide64r64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder)
{
	*pui32Remainder = do_div(ui64Divident, ui32Divisor);

	return ui64Divident;
}

IMG_UINT32 OSDivide64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder)
{
	*pui32Remainder = do_div(ui64Divident, ui32Divisor);

	return (IMG_UINT32) ui64Divident;
}

/* One time osfunc initialisation */
PVRSRV_ERROR PVROSFuncInit(void)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	{
		PVR_ASSERT(!psTimerWorkQueue);

		psTimerWorkQueue = create_workqueue("pvr_timer" PVRSRV_GPUVIRT_OSID_STR);
		if (psTimerWorkQueue == NULL)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: couldn't create timer workqueue", __FUNCTION__));
			return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;
		}
	}
#endif

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	{
		IMG_UINT32 ui32i;

		for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
		{
			TIMER_CALLBACK_DATA *psTimerCBData = &sTimers[ui32i];

			INIT_WORK(&psTimerCBData->sWork, OSTimerWorkQueueCallBack);
		}
	}
#endif
	return PVRSRV_OK;
}

/*
 * Osfunc deinitialisation.
 * Note that PVROSFuncInit may not have been called
 */
void PVROSFuncDeInit(void)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	if (psTimerWorkQueue != NULL)
	{
		destroy_workqueue(psTimerWorkQueue);
		psTimerWorkQueue = NULL;
	}
#endif
}

void OSDumpStack(void)
{
	dump_stack();
}

static struct task_struct *gsOwner;

void OSAcquireBridgeLock(void)
{
	mutex_lock(&gPVRSRVLock);
	gsOwner = current;
}

void OSReleaseBridgeLock(void)
{
	gsOwner = NULL;
	mutex_unlock(&gPVRSRVLock);
}

struct task_struct *OSGetBridgeLockOwner(void)
{
	return gsOwner;
}

static struct task_struct *gsPMRLockOwner;

void PMRLock(void)
{
	OSLockAcquire(&gGlobalLookupPMRLock);
	gsPMRLockOwner = current;
}

void PMRUnlock(void)
{
	gsPMRLockOwner = NULL;
	OSLockRelease(&gGlobalLookupPMRLock);
}

static struct task_struct *OSGetPMRLockOwner(void)
{
	return gsPMRLockOwner;
}

IMG_BOOL PMRIsLocked(void)
{
	return OSLockIsLocked(&gGlobalLookupPMRLock);
}

IMG_BOOL PMRIsLockedByMe(void)
{
	return (OSGetPMRLockOwner() == current);
}


/*************************************************************************/ /*!
@Function		OSCreateStatisticEntry
@Description	Create a statistic entry in the specified folder.
@Input			pszName		   String containing the name for the entry.
@Input			pvFolder	   Reference from OSCreateStatisticFolder() of the
							   folder to create the entry in, or NULL for the
							   root.
@Input			pfnStatsPrint  Pointer to function that can be used to print the
							   values of all the statistics.
@Input			pfnIncMemRefCt Pointer to function that can be used to take a
							   reference on the memory backing the statistic
							   entry.
@Input			pfnDecMemRefCt Pointer to function that can be used to drop a
							   reference on the memory backing the statistic
							   entry.
@Input			pvData		   OS specific reference that can be used by
							   pfnGetElement.
@Return			Pointer void reference to the entry created, which can be
				passed to OSRemoveStatisticEntry() to remove the entry.
*/ /**************************************************************************/
void *OSCreateStatisticEntry(IMG_CHAR* pszName, void *pvFolder,
							 OS_STATS_PRINT_FUNC* pfnStatsPrint,
							 OS_INC_STATS_MEM_REFCOUNT_FUNC* pfnIncMemRefCt,
							 OS_DEC_STATS_MEM_REFCOUNT_FUNC* pfnDecMemRefCt,
							 void *pvData)
{
	return (void *)PVRDebugFSCreateStatisticEntry(pszName, (PVR_DEBUGFS_DIR_DATA *)pvFolder, pfnStatsPrint, pfnIncMemRefCt, pfnDecMemRefCt, pvData);
} /* OSCreateStatisticEntry */


/*************************************************************************/ /*!
@Function		OSRemoveStatisticEntry
@Description	Removes a statistic entry.
@Input			pvEntry  Pointer void reference to the entry created by
						 OSCreateStatisticEntry().
*/ /**************************************************************************/
void OSRemoveStatisticEntry(void *pvEntry)
{
	PVRDebugFSRemoveStatisticEntry((PVR_DEBUGFS_DRIVER_STAT *)pvEntry);
} /* OSRemoveStatisticEntry */


/*************************************************************************/ /*!
@Function		OSCreateStatisticFolder
@Description	Create a statistic folder to hold statistic entries.
@Input			pszName   String containing the name for the folder.
@Input			pvFolder  Reference from OSCreateStatisticFolder() of the folder
						  to create the folder in, or NULL for the root.
@Return			Pointer void reference to the folder created, which can be
				passed to OSRemoveStatisticFolder() to remove the folder.
*/ /**************************************************************************/
void *OSCreateStatisticFolder(IMG_CHAR *pszName, void *pvFolder)
{
	PVR_DEBUGFS_DIR_DATA *psNewStatFolder = NULL;
	int iResult;

	iResult = PVRDebugFSCreateEntryDir(pszName, (PVR_DEBUGFS_DIR_DATA *)pvFolder, &psNewStatFolder);
	return (iResult == 0) ? (void *)psNewStatFolder : NULL;
} /* OSCreateStatisticFolder */


/*************************************************************************/ /*!
@Function		OSRemoveStatisticFolder
@Description	Removes a statistic folder.
@Input          ppvFolder  Reference from OSCreateStatisticFolder() of the
                           folder that should be removed.
                           This needs to be double pointer because it has to
                           be NULLed right after memory is freed to avoid
                           possible races and use-after-free situations.
*/ /**************************************************************************/
void OSRemoveStatisticFolder(void **ppvFolder)
{
	PVRDebugFSRemoveEntryDir((PVR_DEBUGFS_DIR_DATA **)ppvFolder);
} /* OSRemoveStatisticFolder */


/*************************************************************************/ /*!
@Function		OSChangeSparseMemCPUAddrMap
@Description    This function changes the CPU map of the underlying sparse
				allocation.
@return			PVRSRV_OK on success & error code on failure.
*/ /**************************************************************************/
PVRSRV_ERROR OSChangeSparseMemCPUAddrMap(void **psPageArray,
		IMG_UINT64 sCpuVAddrBase,
		uintptr_t sCpuPAHeapBase,
		IMG_UINT32 ui32AllocPageCount,
		IMG_UINT32 *pai32AllocIndices,
		IMG_UINT32 ui32FreePageCount,
		IMG_UINT32 *pai32FreeIndices,
		IMG_UINT32	*pui32Status,
		IMG_BOOL bIsLMA)
{
	int eError = ~PVRSRV_OK;
	struct mm_struct  *psMM= current->mm;
	struct vm_area_struct *psVMA=NULL;
	IMG_UINT64 uiPFN = 0, uiCPUVirtAddr=0;
	IMG_UINT32	ui32Loop=0, ui32PageSize = OSGetPageSize();
	struct address_space *mapping = NULL;
	IMG_BOOL bMixedMap = IMG_FALSE;
	struct page *page = NULL;
	PVR_UNREFERENCED_PARAMETER(pui32Status);

	/*
	 * Acquire the lock before manipulating the VMA
	 * In this case only mmap_sem lock would suffice as the pages associated with this VMA
	 * are never meant to be swapped out
	 *
	 * In future in case the pages are marked as swapped, page_table_lock need to be acquired
	 * in conjunction with this to stop the swap out of the pages
	 */
	/*Find the Virtual Memory Area associated with the user base address */
	psVMA = find_vma(psMM, (uintptr_t)sCpuVAddrBase);
	if(NULL == psVMA)
	{
		eError = PVRSRV_ERROR_PMR_NO_CPU_MAP_FOUND;
		return eError;
	}

	/*Acquire the memory sem */
	down_write(&psMM->mmap_sem);

	mapping = psVMA->vm_file->f_mapping;
	
	/*Set the page offset to the correct value as this is disturbed in MMAP_PMR func*/
	psVMA->vm_pgoff = (psVMA->vm_start >>  PAGE_SHIFT);

	/*Delete the entries for the pages that got freed */
	if(ui32FreePageCount && (pai32FreeIndices != NULL))
	{
		for(ui32Loop = 0; ui32Loop < ui32FreePageCount; ui32Loop++)
		{
			uiCPUVirtAddr = (uintptr_t)(sCpuVAddrBase+(pai32FreeIndices[ui32Loop] * ui32PageSize));

			unmap_mapping_range(mapping,uiCPUVirtAddr,ui32PageSize,1);


#ifndef PVRSRV_UNMAP_ON_SPARSE_CHANGE
			/*
			 * Still need to map pages in case remap flag is set
			 * That is not done until the remap case succeeds
			 */
#endif
		}
		eError = PVRSRV_OK;
	}

	if((psVMA->vm_flags & VM_MIXEDMAP) || bIsLMA)
	{
		psVMA->vm_flags |=  VM_MIXEDMAP;
		bMixedMap = IMG_TRUE;
	}
	else
	{
		if(ui32AllocPageCount && (NULL != pai32AllocIndices))
		{
			for(ui32Loop = 0; ui32Loop < ui32AllocPageCount; ui32Loop++)
			{

				page = (struct page *)psPageArray[pai32AllocIndices[ui32Loop]];
				uiPFN = page_to_pfn((struct page *)psPageArray[pai32AllocIndices[ui32Loop]]);

				if(!pfn_valid(uiPFN) || (page_count(pfn_to_page(uiPFN)) == 0))
				{
					bMixedMap = IMG_TRUE;
					psVMA->vm_flags |= VM_MIXEDMAP;
					break;
				}
			}
		}
	}

	/*Map the pages that got allocated */
	if(ui32AllocPageCount && (NULL != pai32AllocIndices))
	{
		for(ui32Loop = 0; ui32Loop < ui32AllocPageCount; ui32Loop++)
		{
			uiCPUVirtAddr = (uintptr_t)(sCpuVAddrBase+(pai32AllocIndices[ui32Loop] * ui32PageSize));

			unmap_mapping_range(mapping,uiCPUVirtAddr,ui32PageSize, 1);
			if(bIsLMA)
			{
				uiPFN = sCpuPAHeapBase+((IMG_DEV_PHYADDR *)psPageArray)[pai32AllocIndices[ui32Loop]].uiAddr;
				uiPFN  = uiPFN >> PAGE_SHIFT;
				page = pfn_to_page(uiPFN);
			}else{
				page = (struct page *)psPageArray[pai32AllocIndices[ui32Loop]];
				uiPFN = page_to_pfn((struct page *)psPageArray[pai32AllocIndices[ui32Loop]]);
			}

			if(bMixedMap )
			{
				eError = vm_insert_mixed(psVMA,uiCPUVirtAddr, uiPFN);
			}
			else
			{
				eError = vm_insert_page(psVMA,uiCPUVirtAddr,page);
			}

			if(0 != eError)
			{
				PVR_DPF((PVR_DBG_MESSAGE,"Remap failure error code: %d", eError));
				eError = PVRSRV_ERROR_PMR_CPU_PAGE_MAP_FAILED;
				goto eFailed;
			}
		}
	}
	eError = PVRSRV_OK;
	eFailed:
	up_write(&psMM->mmap_sem);
	return eError;
}
