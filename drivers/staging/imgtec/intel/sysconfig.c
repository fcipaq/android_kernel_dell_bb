/*************************************************************************/ /*!
@File           
@Title          System Configuration
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    System Configuration functions
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

#include <drm/drmP.h>
#include "img_types.h"
#include "pwr_mgmt.h"
#include "psb_irq.h"
#include "pvrsrv_device.h"
#include "syscommon.h"
#include "sysconfig.h"
#include "allocmem.h"
#include "pvr_debug.h"
#include "osfunc.h"

#include "pci_support.h"

#include "dfrgx_interface.h"

#if defined(SUPPORT_ION)
#include PVR_ANDROID_ION_HEADER
#include "ion_support.h"
#endif

typedef struct _PLAT_DATA_
{
	IMG_HANDLE	hRGXPCI;

	struct drm_device *psDRMDev;
} PLAT_DATA;

extern struct drm_device *gpsPVRDRMDev;

/* Unused globals to keep link with 3rdparty components happy */
IMG_BOOL gbSystemActivePMEnabled;
IMG_BOOL gbSystemActivePMInit;


PVRSRV_ERROR SysDevInit(void *pvOSDevice, PVRSRV_DEVICE_CONFIG **ppsDevConfig)
{
	PVRSRV_DEVICE_CONFIG *psDevice = &sDevices[0];
	PLAT_DATA *psPlatData;
	IMG_UINT32 ui32MaxOffset;
	IMG_UINT32 ui32BaseAddr = 0;
	PVRSRV_ERROR eError;

	if (psDevice->pvOSDevice)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	psPlatData = OSAllocZMem(sizeof(*psPlatData));
	if (!psPlatData)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psDevice->hSysData = (IMG_HANDLE) psPlatData;

	psPlatData->psDRMDev = gpsPVRDRMDev;
	if (!psPlatData->psDRMDev)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: DRM device not initialized", __func__));
		eError = PVRSRV_ERROR_NOT_SUPPORTED;
		goto ErrorFreePlatData;
	}

	if (!IS_MRFLD(psPlatData->psDRMDev))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Device 0x%08x not supported", __func__,
				 psPlatData->psDRMDev->pci_device));
		eError = PVRSRV_ERROR_NOT_SUPPORTED;
		goto ErrorFreePlatData;
	}

	psPlatData->hRGXPCI = OSPCISetDev((void *)psPlatData->psDRMDev->pdev, 0);
	if (!psPlatData->hRGXPCI)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire PCI device", __func__));
		eError = PVRSRV_ERROR_PCI_DEVICE_NOT_FOUND;
		goto ErrorFreePlatData;
	}

	ui32MaxOffset = OSPCIAddrRangeLen(psPlatData->hRGXPCI, 0);
	if (ui32MaxOffset < (RGX_REG_OFFSET + RGX_REG_SIZE))
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: Device memory region 0x%08x isn't big enough",
				 __func__, ui32MaxOffset));
		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto ErrorFreePlatData;
	}
	PVR_DPF((PVR_DBG_WARNING, "%s: Device memory region len 0x%08x",
			 __func__, ui32MaxOffset));

	/* Reserve the address range */
	if (OSPCIRequestAddrRange(psPlatData->hRGXPCI, 0) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Device memory region not available",
				 __func__));
		eError = PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		goto ErrorFreePlatData;

	}

	ui32BaseAddr = OSPCIAddrRangeStart(psPlatData->hRGXPCI, 0);

	if (OSPCIIRQ(psPlatData->hRGXPCI, &psDevice->ui32IRQ) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Couldn't get IRQ", __func__));
		eError = PVRSRV_ERROR_INVALID_DEVICE;
		goto e4;
	}
	PVR_DPF((PVR_DBG_WARNING, "%s: BaseAddr 0x%08x, EndAddr 0x%llx, IRQ %d",
			 __func__, ui32BaseAddr, OSPCIAddrRangeEnd(psPlatData->hRGXPCI, 0),
			 psDevice->ui32IRQ));

	psDevice->sRegsCpuPBase.uiAddr = ui32BaseAddr + RGX_REG_OFFSET;
	psDevice->ui32RegsSize = RGX_REG_SIZE;
	PVR_DPF((PVR_DBG_WARNING, "%s: sRegsCpuPBase 0x%llx, size 0x%x",
			__func__, psDevice->sRegsCpuPBase.uiAddr, psDevice->ui32RegsSize));

	/* Setup other system specific stuff */
#if defined(SUPPORT_ION)
	IonInit(NULL);
#endif

	psDevice->pvOSDevice = pvOSDevice;

	*ppsDevConfig = psDevice;

	return PVRSRV_OK;

e4:
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI, 0);
	OSPCIReleaseDev(psPlatData->hRGXPCI);

ErrorFreePlatData:
	OSFreeMem(psPlatData);
	psDevice->hSysData = NULL;

	return eError;
}

void SysDevDeInit(PVRSRV_DEVICE_CONFIG *psDevConfig)
{
	PLAT_DATA *psPlatData = (PLAT_DATA *) psDevConfig->hSysData;

	OSPCIReleaseAddrRange(psPlatData->hRGXPCI, 0);
	OSPCIReleaseDev(psPlatData->hRGXPCI);

#if defined(SUPPORT_ION)
	IonDeinit();
#endif

	OSFreeMem(psPlatData);
	psDevConfig->hSysData = NULL;

	psDevConfig->pvOSDevice = NULL;
}

PVRSRV_ERROR SysDebugInfo(PVRSRV_DEVICE_CONFIG *psDevConfig,
				DUMPDEBUG_PRINTF_FUNC *pfnDumpDebugPrintf,
				void *pvDumpDebugFile)
{
	PVR_UNREFERENCED_PARAMETER(psDevConfig);
	PVR_UNREFERENCED_PARAMETER(pfnDumpDebugPrintf);
	PVR_UNREFERENCED_PARAMETER(pvDumpDebugFile);
	return PVRSRV_OK;
}

static void SysCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
								  IMG_UINT32 ui32NumOfAddr,
								  IMG_DEV_PHYADDR *psDevPAddr,
								  IMG_CPU_PHYADDR *psCpuPAddr)
{
	psDevPAddr[0].uiAddr = psCpuPAddr[0].uiAddr;
	if (ui32NumOfAddr > 1)
	{
		IMG_UINT32 ui32Idx;
		for (ui32Idx = 1; ui32Idx < ui32NumOfAddr; ++ui32Idx)
		{
			psDevPAddr[ui32Idx].uiAddr = psCpuPAddr[ui32Idx].uiAddr;
		}
	}
}

static void SysDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
								  IMG_UINT ui32NumOfAddr,
								  IMG_CPU_PHYADDR *psCpuPAddr,
								  IMG_DEV_PHYADDR *psDevPAddr)
{
	psCpuPAddr[0].uiAddr = psDevPAddr[0].uiAddr;
	if (ui32NumOfAddr > 1)
	{
		IMG_UINT32 ui32Idx;
		for (ui32Idx = 1; ui32Idx < ui32NumOfAddr; ++ui32Idx)
		{
			psCpuPAddr[ui32Idx].uiAddr = psDevPAddr[ui32Idx].uiAddr;
		}
	}
}

static PVRSRV_ERROR SysDevicePrePowerState(
		IMG_HANDLE hSysData,
		PVRSRV_DEV_POWER_STATE eNewPowerState,
		PVRSRV_DEV_POWER_STATE eCurrentPowerState,
		IMG_BOOL bForced)
{
	if ((eNewPowerState != eCurrentPowerState) &&
		(eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF))
	{
		PVR_DPF((PVR_DBG_MESSAGE, "Remove SGX power"));

		if (!power_island_put(OSPM_GRAPHICS_ISLAND))
			return PVRSRV_ERROR_DEVICE_POWER_CHANGE_FAILURE;

		/*Report dfrgx We have the device OFF*/
		dfrgx_interface_power_state_set(0);
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR SysDevicePostPowerState(
		IMG_HANDLE hSysData,
		PVRSRV_DEV_POWER_STATE eNewPowerState,
		PVRSRV_DEV_POWER_STATE eCurrentPowerState,
		IMG_BOOL bForced)
{
	if ((eNewPowerState != eCurrentPowerState) &&
		(eCurrentPowerState == PVRSRV_DEV_POWER_STATE_OFF))
	{
		PVR_DPF((PVR_DBG_MESSAGE, "Restore SGX power"));

		if (!power_island_get(OSPM_GRAPHICS_ISLAND))
			return PVRSRV_ERROR_DEVICE_POWER_CHANGE_FAILURE;

		/*Report dfrgx We have the device ON*/
		dfrgx_interface_power_state_set(1);
	}

	return PVRSRV_OK;
}

typedef int (*psb_irq_handler_t)(void *data);

PVRSRV_ERROR SysInstallDeviceLISR(IMG_HANDLE hSysData,
				  IMG_UINT32 ui32IRQ,
				  const IMG_CHAR *pszName,
				  PFN_LISR pfnLISR,
				  void *pvData,
				  IMG_HANDLE *phLISRData)
{
	register_rgx_irq_handler((psb_irq_handler_t) pfnLISR, pvData);
	return PVRSRV_OK;

}

PVRSRV_ERROR SysUninstallDeviceLISR(IMG_HANDLE hLISRData)
{
	register_rgx_irq_handler(NULL, NULL);
	return PVRSRV_OK;

}

/******************************************************************************
 End of file (sysconfig.c)
******************************************************************************/
