/*************************************************************************/ /*!
@File           cache_generic.c
@Title          CPU generic cache management
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements server side code for CPU cache management in a
                CPU agnostic manner.
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
#include "cache_generic.h"
#include "cache_internal.h"
#include "device.h"
#include "pvr_debug.h"
#include "pvrsrv.h"
#include "osfunc.h"
#include "pmr.h"

#if defined(SUPPORT_RANGEBASED_CACHEFLUSH)
//#define CACHEOP_NO_CACHE_LINE_ALIGNED_ROUNDING
static IMG_UINT32 guiCacheLineSize = 0;
static size_t guiOSPageSize = 0;

/* Perform requested CacheOp on the CPU data cache for successive cache
   line worth of bytes up to page or in-page cache-line boundary */
static INLINE void CacheOpCPURangeBased (PVRSRV_CACHE_OP uiCacheOp,
										 IMG_BYTE *pbCpuVirtAddr,
										 IMG_CPU_PHYADDR sCpuPhyAddr,
										 IMG_DEVMEM_OFFSET_T uiPgAlignedOffset,
										 IMG_DEVMEM_OFFSET_T uiCLAlignedStartOffset,
										 IMG_DEVMEM_OFFSET_T uiCLAlignedEndOffset)
{
	IMG_BYTE *pbCpuVirtAddrEnd;
	IMG_BYTE *pbCpuVirtAddrStart;
	IMG_CPU_PHYADDR sCpuPhyAddrEnd;
	IMG_CPU_PHYADDR sCpuPhyAddrStart;
	IMG_DEVMEM_SIZE_T uiRelFlushSize;
	IMG_DEVMEM_OFFSET_T uiRelFlushOffset;
	IMG_DEVMEM_SIZE_T uiNextPgAlignedOffset;

	/* These quantities allows us to perform cache operations
	   at cache-line granularity thereby ensuring we do not
	   perform more than is necessary */
	PVR_ASSERT(uiPgAlignedOffset < uiCLAlignedEndOffset);
	uiRelFlushSize = (IMG_DEVMEM_SIZE_T)guiOSPageSize;
	uiRelFlushOffset = 0;

	if (uiCLAlignedStartOffset > uiPgAlignedOffset)
	{
		/* Zero unless initially starting at an in-page offset */
		uiRelFlushOffset = uiCLAlignedStartOffset - uiPgAlignedOffset;
		uiRelFlushSize -= uiRelFlushOffset;
	}

	/* uiRelFlushSize is guiOSPageSize unless current outstanding CacheOp
	   size is smaller. The 1st case handles in-page CacheOp range and
	   the 2nd case handles multiple-page CacheOp range with a last
	   CacheOp size that is less than guiOSPageSize */
	uiNextPgAlignedOffset = uiPgAlignedOffset + (IMG_DEVMEM_SIZE_T)guiOSPageSize;
	if (uiNextPgAlignedOffset < uiPgAlignedOffset)
	{
		/* uiNextPgAlignedOffset is greater than uiCLAlignedEndOffset
		   by implication of this wrap-round; this only happens when
		   uiPgAlignedOffset is the last page aligned offset */
		uiRelFlushSize = uiRelFlushOffset ?
				uiCLAlignedEndOffset - uiCLAlignedStartOffset :
				uiCLAlignedEndOffset - uiPgAlignedOffset;
	}
	else
	{
		if (uiNextPgAlignedOffset > uiCLAlignedEndOffset)
		{
			uiRelFlushSize = uiRelFlushOffset ?
					uiCLAlignedEndOffset - uiCLAlignedStartOffset :
					uiCLAlignedEndOffset - uiPgAlignedOffset;
		}
	}

	/* More efficient to request cache maintenance operation for full
	   relative range as opposed to multiple cache-aligned ranges */
	pbCpuVirtAddrStart = pbCpuVirtAddr + uiRelFlushOffset;
	pbCpuVirtAddrEnd = pbCpuVirtAddrStart + uiRelFlushSize;
	sCpuPhyAddrStart.uiAddr = sCpuPhyAddr.uiAddr + uiRelFlushOffset;
	sCpuPhyAddrEnd.uiAddr = sCpuPhyAddrStart.uiAddr + uiRelFlushSize;

	switch (uiCacheOp)
	{
		case PVRSRV_CACHE_OP_CLEAN:
			OSCleanCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
									sCpuPhyAddrStart, sCpuPhyAddrEnd);
			break;
		case PVRSRV_CACHE_OP_INVALIDATE:
			OSInvalidateCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
									sCpuPhyAddrStart, sCpuPhyAddrEnd);
			break;
		case PVRSRV_CACHE_OP_FLUSH:
			OSFlushCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
									sCpuPhyAddrStart, sCpuPhyAddrEnd);
			break;
		default:
			PVR_DPF((PVR_DBG_ERROR,	"%s: Invalid cache operation type %d",
					__FUNCTION__, uiCacheOp));
			PVR_ASSERT(0);
			break;
	}
}

PVRSRV_ERROR CacheOpQueue(PMR *psPMR,
						  IMG_DEVMEM_OFFSET_T uiOffset,
						  IMG_DEVMEM_SIZE_T uiSize,
						  PVRSRV_CACHE_OP uiCacheOp)
{
	IMG_HANDLE hPrivOut;
	IMG_BOOL bPMRIsSparse;
	IMG_UINT32 ui32PageIndex;
	IMG_UINT32 ui32NumOfPages;
	IMG_DEVMEM_SIZE_T uiOutSize;
	IMG_DEVMEM_SIZE_T uiPgAlignedSize;
	IMG_DEVMEM_OFFSET_T uiCLAlignedEndOffset;
	IMG_DEVMEM_OFFSET_T uiPgAlignedEndOffset;
	IMG_DEVMEM_OFFSET_T uiCLAlignedStartOffset;
	IMG_DEVMEM_OFFSET_T uiPgAlignedStartOffset;
	IMG_DEVMEM_OFFSET_T uiPgAlignedOffsetNext;
	PVRSRV_CACHE_OP_ADDR_TYPE uiCacheOpAddrType;
	IMG_BOOL abValid[PMR_MAX_TRANSLATION_STACK_ALLOC];
	IMG_CPU_PHYADDR asCpuPhyAddr[PMR_MAX_TRANSLATION_STACK_ALLOC];
	IMG_UINT32 OS_PAGE_SHIFT = (IMG_UINT32) OSGetPageShift();
	IMG_CPU_PHYADDR *psCpuPhyAddr = asCpuPhyAddr;
	IMG_BOOL bIsPMRDataRetrieved = IMG_FALSE;
	PVRSRV_ERROR eError = PVRSRV_OK;
	IMG_BYTE *pbCpuVirtAddr = NULL;
	IMG_BOOL *pbValid = abValid;

	if (uiCacheOp == PVRSRV_CACHE_OP_NONE)
	{
		PVR_ASSERT(0);
		return PVRSRV_OK;
	}
	else
	{
		/* Carry out full dcache operation if size (in pages) qualifies */
		if ((uiSize >> OS_PAGE_SHIFT) > PVR_DIRTY_PAGECOUNT_FLUSH_THRESHOLD)
		{
			eError = OSCPUOperation(PVRSRV_CACHE_OP_FLUSH);
			if (eError == PVRSRV_OK)
			{
				return PVRSRV_OK;
			}
		}

		if (! guiCacheLineSize)
		{
			guiCacheLineSize = OSCPUCacheAttributeSize(PVR_DCACHE_LINE_SIZE);
			PVR_ASSERT(guiCacheLineSize != 0);

			guiOSPageSize = OSGetPageSize();
			PVR_ASSERT(guiOSPageSize != 0);
		}
	}

	/* Need this for kernel mapping */
	bPMRIsSparse = PMR_IsSparse(psPMR);

	/* Round the incoming offset down to the nearest cache-line / page aligned-address */
	uiCLAlignedEndOffset = uiOffset + uiSize;
	uiCLAlignedEndOffset = PVR_ALIGN(uiCLAlignedEndOffset, (IMG_DEVMEM_SIZE_T)guiCacheLineSize);
	uiCLAlignedStartOffset = (uiOffset & ~((IMG_DEVMEM_OFFSET_T)guiCacheLineSize-1));

	uiPgAlignedEndOffset = uiCLAlignedEndOffset;
	uiPgAlignedEndOffset = PVR_ALIGN(uiPgAlignedEndOffset, (IMG_DEVMEM_SIZE_T)guiOSPageSize);
	uiPgAlignedStartOffset = (uiOffset & ~((IMG_DEVMEM_OFFSET_T)guiOSPageSize-1));
	uiPgAlignedSize = uiPgAlignedEndOffset - uiPgAlignedStartOffset;

#if defined(CACHEOP_NO_CACHE_LINE_ALIGNED_ROUNDING)
	/* For internal debug if cache-line optimised
	   flushing is suspected of causing data corruption */
	uiCLAlignedStartOffset = uiPgAlignedStartOffset;
	uiCLAlignedEndOffset = uiPgAlignedEndOffset;
#endif

	/* Which type of address(es) do we need for this CacheOp */
	uiCacheOpAddrType = OSCPUCacheOpAddressType(uiCacheOp);

	/* Type of allocation backing the PMR data */
	ui32NumOfPages = uiPgAlignedSize >> OS_PAGE_SHIFT;
	if (ui32NumOfPages > PMR_MAX_TRANSLATION_STACK_ALLOC)
	{
		/* The pbValid array is allocated first as it is needed in
		   both physical/virtual cache maintenance methods */
		pbValid = OSAllocZMem(ui32NumOfPages * sizeof(IMG_BOOL));
		if (pbValid != NULL)
		{
			if (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_VIRTUAL)
			{
				psCpuPhyAddr = OSAllocZMem(ui32NumOfPages * sizeof(IMG_CPU_PHYADDR));
				if (psCpuPhyAddr == NULL)
				{
					psCpuPhyAddr = asCpuPhyAddr;
					OSFreeMem(pbValid);
					pbValid = abValid;
				}
			}
		}
		else
		{
			pbValid = abValid;
		}
	}

	/* We always retrieve PMR data in bulk, up-front if number of pages is within
	   PMR_MAX_TRANSLATION_STACK_ALLOC limits else we check to ensure that a 
	   dynamic buffer has been allocated to satisfy requests outside limits */
	if (ui32NumOfPages <= PMR_MAX_TRANSLATION_STACK_ALLOC || pbValid != abValid)
	{
		if (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_VIRTUAL)
		{
			/* Look-up PMR CpuPhyAddr once, if possible */
			eError = PMR_CpuPhysAddr(psPMR,
									 OS_PAGE_SHIFT,
									 ui32NumOfPages,
									 uiPgAlignedStartOffset,
									 psCpuPhyAddr,
									 pbValid);
			if (eError == PVRSRV_OK)
			{
				bIsPMRDataRetrieved = IMG_TRUE;
			}
		}
	}

	/* For each device page, carry out the requested cache maintenance operation */
	for (uiPgAlignedOffsetNext = uiPgAlignedStartOffset, ui32PageIndex = 0;
		 uiPgAlignedOffsetNext < uiPgAlignedEndOffset;
		 uiPgAlignedOffsetNext += (IMG_DEVMEM_OFFSET_T) guiOSPageSize, ui32PageIndex += 1)
	{
		if (bIsPMRDataRetrieved == IMG_FALSE)
		{
			/* Never cross page boundary without looking up corresponding
			   PMR page physical address and/or page validity if these
			   were not looked-up, in bulk, up-front */	
			ui32PageIndex = 0;
			if (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_VIRTUAL)
			{
				eError = PMR_CpuPhysAddr(psPMR,
										 OS_PAGE_SHIFT,
										 1,
										 uiPgAlignedOffsetNext,
										 psCpuPhyAddr,
										 pbValid);
				if (eError != PVRSRV_OK)
				{
					PVR_ASSERT(0);
					goto e0;
				}
			}
			else
			{
				PMR_IsOffsetValid(psPMR,
								  uiPgAlignedOffsetNext,
								  pbValid);
			}
		}

		/* Skip invalid PMR pages (i.e. sparse) */
		if (pbValid[ui32PageIndex] == IMG_FALSE)
		{
			continue;
		}

		/* Skip virtual address acquire if CacheOp can be maintained
		   entirely using PMR physical addresses */
		if (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_PHYSICAL)
		{
			if (bPMRIsSparse)
			{
				eError =
					PMRAcquireSparseKernelMappingData(psPMR,
													  uiPgAlignedOffsetNext,
													  guiOSPageSize,
													  (void **)&pbCpuVirtAddr,
													  (size_t*)&uiOutSize,
													  &hPrivOut);
				if (eError != PVRSRV_OK)
				{
					PVR_ASSERT(0);
					goto e0;
				}
			}
			else
			{
				eError =
					PMRAcquireKernelMappingData(psPMR,
												uiPgAlignedOffsetNext,
												guiOSPageSize,
												(void **)&pbCpuVirtAddr,
												(size_t*)&uiOutSize,
												&hPrivOut);
				if (eError != PVRSRV_OK)
				{
					PVR_ASSERT(0);
					goto e0;
				}
			}
		}

		/* Issue actual cache maintenance for PMR */
		CacheOpCPURangeBased(uiCacheOp,
							 pbCpuVirtAddr,
							 (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_VIRTUAL) ?
								psCpuPhyAddr[ui32PageIndex] : psCpuPhyAddr[0],
							 uiPgAlignedOffsetNext,
							 uiCLAlignedStartOffset,
							 uiCLAlignedEndOffset);

		/* Skip virtual address release if CacheOp can be maintained
		   entirely using PMR physical addresses */
		if (uiCacheOpAddrType != PVRSRV_CACHE_OP_ADDR_TYPE_PHYSICAL)
		{
			eError = PMRReleaseKernelMappingData(psPMR, hPrivOut);
			PVR_ASSERT(eError == PVRSRV_OK);
		}
	}

e0:
	if (psCpuPhyAddr != asCpuPhyAddr)
	{
		OSFreeMem(psCpuPhyAddr);
	}

	if (pbValid != abValid)
	{
		OSFreeMem(pbValid);
	}

	return eError;
}
#else
PVRSRV_ERROR CacheOpQueue(PMR *psPMR,
						  IMG_DEVMEM_OFFSET_T uiOffset,
						  IMG_DEVMEM_SIZE_T uiSize,
						  PVRSRV_CACHE_OP uiCacheOp)
{
	PVRSRV_DATA *psData = PVRSRVGetPVRSRVData();
	psData->uiCacheOp = SetCacheOp(psData->uiCacheOp, uiCacheOp);
	return PVRSRV_OK;
}
#endif
