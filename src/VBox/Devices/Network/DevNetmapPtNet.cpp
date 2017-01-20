/* $Id: DevNetmapPtNet.cpp $ */
/** @file
 * DevNetmapPtNet - Netmap passthrough device.
 *
 */

/*
 * Copyright (C) 2016 Vincenzo Maffione
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_PTNET
#include <iprt/crc.h>
#include <iprt/ctype.h>
#include <iprt/net.h>
#include <iprt/semaphore.h>
#include <iprt/string.h>
#include <iprt/time.h>
#include <iprt/uuid.h>
#include <VBox/vmm/pdmdev.h>
#include <VBox/vmm/pdmnetifs.h>
#include <VBox/param.h>
#include "VBoxDD.h"


#ifndef DEBUG
#define PtnetLog(a)     LogRel(a)
#else
#define PtnetLog(a)     Log(a)
#endif


/**
 * Device state structure.
 *
 * Holds the current state of device.
 *
 * @implements  PDMINETWORKDOWN
 * @implements  PDMINETWORKCONFIG
 */
struct PtnetState_st
{
    char                    szPrf[8];                /**< Log prefix, e.g. PTNET#1. */
    PDMIBASE                IBase;
    PDMINETWORKDOWN         INetworkDown;
    PDMINETWORKCONFIG       INetworkConfig;
    R3PTRTYPE(PPDMIBASE)    pDrvBase;                 /**< Attached network driver. */

    PPDMDEVINSR3            pDevInsR3;                   /**< Device instance - R3. */
    R3PTRTYPE(PPDMQUEUE)    pTxQueueR3;                   /**< Transmit queue - R3. */
    R3PTRTYPE(PPDMQUEUE)    pCanRxQueueR3;           /**< Rx wakeup signaller - R3. */
    PPDMINETWORKUPR3        pDrvR3;              /**< Attached network driver - R3. */
    /** The scatter / gather buffer used for the current outgoing packet - R3. */
    R3PTRTYPE(PPDMSCATTERGATHER) pTxSgR3;

    PPDMDEVINSR0            pDevInsR0;                   /**< Device instance - R0. */
    R0PTRTYPE(PPDMQUEUE)    pTxQueueR0;                   /**< Transmit queue - R0. */
    R0PTRTYPE(PPDMQUEUE)    pCanRxQueueR0;           /**< Rx wakeup signaller - R0. */
    PPDMINETWORKUPR0        pDrvR0;              /**< Attached network driver - R0. */
    /** The scatter / gather buffer used for the current outgoing packet - R0. */
    R0PTRTYPE(PPDMSCATTERGATHER) pTxSgR0;

#if HC_ARCH_BITS != 32
    uint32_t                Alignment1;
#endif
    PDMCRITSECT cs;                  /**< Critical section */
    /** Base address of memory-mapped registers. */
    RTGCPHYS    addrMMReg;
    /** MAC address obtained from the configuration. */
    RTMAC       macConfigured;
    /** Base port of I/O space region. */
    RTIOPORT    IOPortBase;
    /** EMT: */
    PDMPCIDEV   pciDevice;
    /** EMT: false if the cable is disconnected by the GUI. */
    bool        fCableConnected;
    /** EMT: */
    bool        fR0Enabled;

    /** All: Device register storage. */
    uint32_t    auRegs[100];

    /** EMT: Gets signalled when XXX happens. */
    RTSEMEVENT hEvent;
};
typedef struct PtnetState_st PTNETST;
/** Pointer to the PTNET device state. */
typedef PTNETST *PPTNETST;

#ifndef VBOX_DEVICE_STRUCT_TESTCASE

#define ptnetCsEnter(ps, rc) PDMCritSectEnter(&ps->cs, rc)
#define ptnetCsLeave(ps) PDMCritSectLeave(&ps->cs)

#ifdef IN_RING3

#include <net/if.h>
#include <net/netmap.h>
#include <net/netmap_virt.h>

/**
 * Wakeup the RX thread.
 */
static void ptnetWakeupReceive(PPDMDEVINS pDevIns)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PPTNETST);
    PtnetLog(("%s Waking up Out-of-RX-space semaphore\n",  pThis->szPrf));
    RTSemEventSignal(pThis->hEvent);
}
#endif /* IN_RING3 */

#if 0
static int ptnetRaiseInterrupt(PPTNETST pThis, int rcBusy, uint32_t u32IntCause = 0)
{
	PDMDevHlpPCISetIrq(pThis->CTX_SUFF(pDevIns), 0, 1);
}
#endif


#ifdef IN_RING3
/**
 * Callback for consuming from transmit queue. It gets called in R3 whenever
 * we enqueue something in R0/GC.
 *
 * @returns true
 * @param   pDevIns     Pointer to device instance structure.
 * @param   pItem       Pointer to the element being dequeued (not used).
 * @thread  ???
 */
static DECLCALLBACK(bool) ptnetTxQueueConsumer(PPDMDEVINS pDevIns, PPDMQUEUEITEMCORE pItem)
{
    NOREF(pItem);
    PPTNETST  pThis = PDMINS_2_DATA(pDevIns, PPTNETST);
    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    return true;
}

/**
 * Handler for the wakeup signaller queue.
 */
static DECLCALLBACK(bool) ptnetCanRxQueueConsumer(PPDMDEVINS pDevIns, PPDMQUEUEITEMCORE pItem)
{
    RT_NOREF(pItem);
    ptnetWakeupReceive(pDevIns);
    return true;
}

#endif /* IN_RING3 */


/* -=-=-=-=- MMIO and I/O Port Callbacks -=-=-=-=- */

/**
 * @callback_method_impl{FNIOMMMIOREAD}
 */
PDMBOTHCBDECL(int) ptnetMMIORead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF2(pvUser, cb);
    PPTNETST pThis  = PDMINS_2_DATA(pDevIns, PPTNETST);
    uint32_t  offReg = GCPhysAddr - pThis->addrMMReg;
    uint32_t *value = (uint32_t *)pv;

    *value = 0U;
    PtnetLog(("%s %s: offReg=%08x cb=%u\n", pThis->szPrf, __func__, offReg, cb));

    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMMMIOWRITE}
 */
PDMBOTHCBDECL(int) ptnetMMIOWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF2(pvUser, cb);
    PPTNETST pThis  = PDMINS_2_DATA(pDevIns, PPTNETST);
    uint32_t offReg = GCPhysAddr - pThis->addrMMReg;
    uint32_t value = *((uint32_t const *)pv);

    PtnetLog(("%s %s: offReg=%08x value=%08x cb=%u\n", pThis->szPrf, __func__, offReg, value, cb));

    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN}
 */
PDMBOTHCBDECL(int) ptnetIOPortIn(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT uPort, uint32_t *pu32, unsigned cb)
{
    PPTNETST   pThis = PDMINS_2_DATA(pDevIns, PPTNETST);
    int         rc = VINF_SUCCESS;
    RT_NOREF_PV(pvUser);

    *pu32 = 0;

    //uPort -= pThis->IOPortBase;
    switch (uPort) {
        default:
            PtnetLog(("%s %s: uPort=%RTiop cb=%u\n", pThis->szPrf, __func__, uPort, cb));
            break;
    }

    return rc;
}


/**
 * @callback_method_impl{FNIOMIOPORTOUT}
 */
PDMBOTHCBDECL(int) ptnetIOPortOut(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT uPort, uint32_t u32, unsigned cb)
{
    PPTNETST   pThis = PDMINS_2_DATA(pDevIns, PPTNETST);
    int         rc = VINF_SUCCESS;
    RT_NOREF_PV(pvUser);

    //uPort -= pThis->IOPortBase;
    switch (uPort) {
        default:
            PtnetLog(("%s %s: uPort=%RTiop value=%08x cb=%u\n", pThis->szPrf, __func__, uPort, u32, cb));
            break;
    }

    return rc;
}

#ifdef IN_RING3

/**
 * @callback_method_impl{FNPCIIOREGIONMAP}
 */
static DECLCALLBACK(int) ptnetMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType)
{
    RT_NOREF(pPciDev, iRegion);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST *);
    int       rc;

    switch (enmType)
    {
        case PCI_ADDRESS_SPACE_IO:
            pThis->IOPortBase = (RTIOPORT)GCPhysAddress;
            rc = PDMDevHlpIOPortRegister(pDevIns, pThis->IOPortBase, cb, NULL /*pvUser*/,
                                         ptnetIOPortOut, ptnetIOPortIn, NULL, NULL, "PTNET");
            if (pThis->fR0Enabled && RT_SUCCESS(rc))
                rc = PDMDevHlpIOPortRegisterR0(pDevIns, pThis->IOPortBase, cb, NIL_RTR0PTR /*pvUser*/,
                                             "ptnetIOPortOut", "ptnetIOPortIn", NULL, NULL, "PTNET");
            break;

        case PCI_ADDRESS_SPACE_MEM:
            pThis->addrMMReg = GCPhysAddress; Assert(!(GCPhysAddress & 7));
            rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress, cb, NULL /*pvUser*/,
                                       IOMMMIO_FLAGS_READ_DWORD | IOMMMIO_FLAGS_WRITE_ONLY_DWORD,
                                       ptnetMMIOWrite, ptnetMMIORead, "PTNET");
            if (pThis->fR0Enabled && RT_SUCCESS(rc))
                rc = PDMDevHlpMMIORegisterR0(pDevIns, GCPhysAddress, cb, NIL_RTR0PTR /*pvUser*/,
                                             "ptnetMMIOWrite", "ptnetMMIORead");
            break;

        default:
            /* We should never get here */
            AssertMsgFailed(("Invalid PCI address space param in map callback"));
            rc = VERR_INTERNAL_ERROR;
            break;
    }
    return rc;
}


/* -=-=-=-=- PDMINETWORKDOWN -=-=-=-=- */

/**
 * @interface_method_impl{PDMINETWORKDOWN,pfnWaitReceiveAvail}
 */
static DECLCALLBACK(int) ptnetR3NetworkDown_WaitReceiveAvail(PPDMINETWORKDOWN pInterface, RTMSINTERVAL cMillies)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkDown);
    NOREF(cMillies);

    PtnetLog(("%s %s\n", pThis->szPrf, __func__));
    //RTSemEventWait(pThis->hEvent, cMillies);
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMINETWORKDOWN,pfnReceive}
 */
static DECLCALLBACK(int) ptnetR3NetworkDown_Receive(PPDMINETWORKDOWN pInterface, const void *pvBuf, size_t cb)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkDown);
    NOREF(pvBuf);
    NOREF(cb);

    PtnetLog(("%s Dropping incoming packet.\n", pThis->szPrf));
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMINETWORKDOWN,pfnXmitPending}
 */
static DECLCALLBACK(void) ptnetR3NetworkDown_XmitPending(PPDMINETWORKDOWN pInterface)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkDown);
    PtnetLog(("%s %s\n", pThis->szPrf, __func__));
}

/* -=-=-=-=- PDMINETWORKCONFIG -=-=-=-=- */

/**
 * @interface_method_impl{PDMINETWORKCONFIG,pfnGetMac}
 */
static DECLCALLBACK(int) ptnetR3GetMac(PPDMINETWORKCONFIG pInterface, PRTMAC pMac)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkConfig);
    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    //TODO get mac from MACHI and MACLO registers
    memset(pMac, 0x4, 6);
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMINETWORKCONFIG,pfnGetLinkState}
 */
static DECLCALLBACK(PDMNETWORKLINKSTATE) ptnetR3GetLinkState(PPDMINETWORKCONFIG pInterface)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkConfig);
    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    return PDMNETWORKLINKSTATE_UP;
}

/**
 * @interface_method_impl{PDMINETWORKCONFIG,pfnSetLinkState}
 */
static DECLCALLBACK(int) ptnetR3SetLinkState(PPDMINETWORKCONFIG pInterface, PDMNETWORKLINKSTATE enmState)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, INetworkConfig);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    switch (enmState)
    {
        case PDMNETWORKLINKSTATE_UP:
	    if (pThis->pDrvR3)
		    pThis->pDrvR3->pfnNotifyLinkChanged(pThis->pDrvR3, PDMNETWORKLINKSTATE_UP);
            break;
        case PDMNETWORKLINKSTATE_DOWN:
        case PDMNETWORKLINKSTATE_DOWN_RESUME:
	    if (pThis->pDrvR3)
		    pThis->pDrvR3->pfnNotifyLinkChanged(pThis->pDrvR3, PDMNETWORKLINKSTATE_DOWN);
            break;
    }
    return VINF_SUCCESS;
}


/* -=-=-=-=- PDMIBASE -=-=-=-=- */

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
static DECLCALLBACK(void *) ptnetR3QueryInterface(struct PDMIBASE *pInterface, const char *pszIID)
{
    PPTNETST pThis = RT_FROM_MEMBER(pInterface, PTNETST, IBase);
    Assert(&pThis->IBase == pInterface);

    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pThis->IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMINETWORKDOWN, &pThis->INetworkDown);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMINETWORKCONFIG, &pThis->INetworkConfig);
    return NULL;
}


/* -=-=-=-=- Saved State -=-=-=-=- */

/**
 * @callback_method_impl{FNSSMDEVLIVEEXEC,Save basic configuration.}
 */
static DECLCALLBACK(int) ptnetLiveExec(PPDMDEVINS pDevIns, PSSMHANDLE pSSM, uint32_t uPass)
{
    RT_NOREF(uPass);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    SSMR3PutMem(pSSM, &pThis->macConfigured, sizeof(pThis->macConfigured));
    return VINF_SSM_DONT_CALL_AGAIN;
}

/**
 * @callback_method_impl{FNSSMDEVSAVEPREP,Synchronize.}
 */
static DECLCALLBACK(int) ptnetSavePrep(PPDMDEVINS pDevIns, PSSMHANDLE pSSM)
{
    RT_NOREF(pSSM);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNSSMDEVSAVEEXEC}
 */
static DECLCALLBACK(int) ptnetSaveExec(PPDMDEVINS pDevIns, PSSMHANDLE pSSM)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    SSMR3PutMem(pSSM, pThis->auRegs, sizeof(pThis->auRegs));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNSSMDEVLOADPREP,Synchronize.}
 */
static DECLCALLBACK(int) ptnetLoadPrep(PPDMDEVINS pDevIns, PSSMHANDLE pSSM)
{
    RT_NOREF(pSSM);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNSSMDEVLOADEXEC}
 */
static DECLCALLBACK(int) ptnetLoadExec(PPDMDEVINS pDevIns, PSSMHANDLE pSSM, uint32_t uVersion, uint32_t uPass)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    NOREF(uVersion);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    if (uPass == SSM_PASS_FINAL)
    {
        /* the state */
        SSMR3GetMem(pSSM, &pThis->auRegs, sizeof(pThis->auRegs));
    	PtnetLog(("%s: State restored\n", pThis->szPrf));
    }
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNSSMDEVLOADDONE, Link status adjustments after loading.}
 */
static DECLCALLBACK(int) ptnetLoadDone(PPDMDEVINS pDevIns, PSSMHANDLE pSSM)
{
    RT_NOREF(pSSM);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    /* Update promiscuous mode */
    if (pThis->pDrvR3)
        pThis->pDrvR3->pfnSetPromiscuousMode(pThis->pDrvR3, 1);

    return VINF_SUCCESS;
}



/* -=-=-=-=- Debug Info + PtnetLog Types -=-=-=-=- */

/**
 * Status info callback.
 *
 * @param   pDevIns     The device instance.
 * @param   pHlp        The output helpers.
 * @param   pszArgs     The arguments.
 */
static DECLCALLBACK(void) ptnetInfo(PPDMDEVINS pDevIns, PCDBGFINFOHLP pHlp, const char *pszArgs)
{
    RT_NOREF(pszArgs);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    /*
     * Show info.
     */
    pHlp->pfnPrintf(pHlp, "PTNET");
}



/* -=-=-=-=- PDMDEVREG -=-=-=-=- */

/**
 * Detach notification.
 *
 * One port on the network card has been disconnected from the network.
 *
 * @param   pDevIns     The device instance.
 * @param   iLUN        The logical unit which is being detached.
 * @param   fFlags      Flags, combination of the PDMDEVATT_FLAGS_* \#defines.
 */
static DECLCALLBACK(void) ptnetR3Detach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
    RT_NOREF(fFlags);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s ptnetR3Detach:\n", pThis->szPrf));

    AssertLogRelReturnVoid(iLUN == 0);
    PDMCritSectEnter(&pThis->cs, VERR_SEM_BUSY);

    /** @todo r=pritesh still need to check if i missed
     * to clean something in this function
     */

    /*
     * Zero some important members.
     */
    pThis->pDrvBase = NULL;
    pThis->pDrvR3 = NULL;
    pThis->pDrvR0 = NIL_RTR0PTR;

    PDMCritSectLeave(&pThis->cs);
}

/**
 * Attach the Network attachment.
 *
 * One port on the network card has been connected to a network.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   iLUN        The logical unit which is being attached.
 * @param   fFlags      Flags, combination of the PDMDEVATT_FLAGS_* \#defines.
 *
 * @remarks This code path is not used during construction.
 */
static DECLCALLBACK(int) ptnetR3Attach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
    RT_NOREF(fFlags);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));

    AssertLogRelReturn(iLUN == 0, VERR_PDM_NO_SUCH_LUN);

    PDMCritSectEnter(&pThis->cs, VERR_SEM_BUSY);

    /*
     * Attach the driver.
     */
    int rc = PDMDevHlpDriverAttach(pDevIns, 0, &pThis->IBase, &pThis->pDrvBase, "Network Port");
    if (RT_SUCCESS(rc))
    {
        pThis->pDrvR3 = PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMINETWORKUP);
        AssertMsgStmt(pThis->pDrvR3, ("Failed to obtain the PDMINETWORKUP interface!\n"),
                      rc = VERR_PDM_MISSING_INTERFACE_BELOW);
        if (RT_SUCCESS(rc))
        {
            PPDMIBASER0 pBaseR0 = PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMIBASER0);
            pThis->pDrvR0 = pBaseR0 ? pBaseR0->pfnQueryInterface(pBaseR0, PDMINETWORKUP_IID) : NIL_RTR0PTR;
        }
    }
    else if (   rc == VERR_PDM_NO_ATTACHED_DRIVER
             || rc == VERR_PDM_CFG_MISSING_DRIVER_NAME)
    {
        /* This should never happen because this function is not called
         * if there is no driver to attach! */
        PtnetLog(("%s No attached driver!\n", pThis->szPrf));
    }

    PDMCritSectLeave(&pThis->cs);
    return rc;

}

/**
 * @copydoc FNPDMDEVPOWEROFF
 */
static DECLCALLBACK(void) ptnetR3PowerOff(PPDMDEVINS pDevIns)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    PtnetLog(("%s\n", pThis->szPrf));
    ptnetWakeupReceive(pDevIns);
}

/**
 * @copydoc FNPDMDEVRESET
 */
static DECLCALLBACK(void) ptnetR3Reset(PPDMDEVINS pDevIns)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    PtnetLog(("%s\n", pThis->szPrf));
}

/**
 * @copydoc FNPDMDEVSUSPEND
 */
static DECLCALLBACK(void) ptnetR3Suspend(PPDMDEVINS pDevIns)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    PtnetLog(("%s\n", pThis->szPrf));
    ptnetWakeupReceive(pDevIns);
}

/**
 * Device relocation callback.
 *
 * When this callback is called the device instance data, and if the
 * device have a GC component, is being relocated, or/and the selectors
 * have been changed. The device must use the chance to perform the
 * necessary pointer relocations and data updates.
 *
 * Before the GC code is executed the first time, this function will be
 * called with a 0 delta so GC pointer calculations can be one in one place.
 *
 * @param   pDevIns     Pointer to the device instance.
 * @param   offDelta    The relocation delta relative to the old location.
 *
 * @remark  A relocation CANNOT fail.
 */
static DECLCALLBACK(void) ptnetR3Relocate(PPDMDEVINS pDevIns, RTGCINTPTR offDelta)
{
    /* XXX probably useless */
    RT_NOREF(offDelta);
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    NOREF(pThis);
}

/**
 * Destruct a device instance.
 *
 * We need to free non-VM resources only.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance data.
 * @thread  EMT
 */
static DECLCALLBACK(int) ptnetR3Destruct(PPDMDEVINS pDevIns)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    PDMDEV_CHECK_VERSIONS_RETURN_QUIET(pDevIns);

    PtnetLog(("%s: %s\n", pThis->szPrf, __func__));
    if (PDMCritSectIsInitialized(&pThis->cs))
    {
        RTSemEventSignal(pThis->hEvent);
        RTSemEventDestroy(pThis->hEvent);
        pThis->hEvent = NIL_RTSEMEVENT;
        PDMR3CritSectDelete(&pThis->cs);
    }
    return VINF_SUCCESS;
}


/**
 * Set PCI configuration space registers.
 *
 * @param   pci         Reference to PCI device structure.
 * @thread  EMT
 */
static DECLCALLBACK(void) ptnetConfigurePciDev(PPDMPCIDEV pPciDev)
{
    /* Configure PCI Device, assume 32-bit mode ******************************/
    PCIDevSetVendorId(pPciDev, PTNETMAP_PCI_VENDOR_ID);
    PCIDevSetDeviceId(pPciDev, PTNETMAP_PCI_NETIF_ID);
    PCIDevSetWord( pPciDev, VBOX_PCI_SUBSYSTEM_VENDOR_ID, 0x0000);
    PCIDevSetWord( pPciDev, VBOX_PCI_SUBSYSTEM_ID, 0x0000);

    PCIDevSetWord( pPciDev, VBOX_PCI_COMMAND,            0x0000);
    /* DEVSEL Timing (medium device), 66 MHz Capable, New capabilities */
    PCIDevSetWord( pPciDev, VBOX_PCI_STATUS,
                   VBOX_PCI_STATUS_DEVSEL_MEDIUM | VBOX_PCI_STATUS_CAP_LIST |  VBOX_PCI_STATUS_66MHZ);
    PCIDevSetByte( pPciDev, VBOX_PCI_REVISION_ID,          0x01);
    /* Ethernet adapter */
    PCIDevSetByte( pPciDev, VBOX_PCI_CLASS_PROG,           0x00);
    PCIDevSetWord( pPciDev, VBOX_PCI_CLASS_DEVICE,       0x0200);
    /* normal single function Ethernet controller */
    PCIDevSetByte( pPciDev, VBOX_PCI_HEADER_TYPE,          0x00);
    /* IO Register Base Address */
    PCIDevSetDWord(pPciDev, VBOX_PCI_BASE_ADDRESS_0, 0x00000001);
    /* XXX Unused, memory */
    PCIDevSetDWord(pPciDev, VBOX_PCI_BASE_ADDRESS_1, 0x00000000);
    /* Memory Register Base Address */
    PCIDevSetDWord(pPciDev, VBOX_PCI_BASE_ADDRESS_2, 0x00000000);
    /* Capabilities Pointer */
    PCIDevSetByte( pPciDev, VBOX_PCI_CAPABILITY_LIST,      0xDC);
    /* Interrupt Pin: INTA# */
    PCIDevSetByte( pPciDev, VBOX_PCI_INTERRUPT_PIN,        0x01);
    /* Max_Lat/Min_Gnt: very high priority and time slice */
    PCIDevSetByte( pPciDev, VBOX_PCI_MIN_GNT,              0xFF);
    PCIDevSetByte( pPciDev, VBOX_PCI_MAX_LAT,              0x00);

    /* PCI Power Management Registers ****************************************/
    /* Capability ID: PCI Power Management Registers */
    PCIDevSetByte( pPciDev, 0xDC,            VBOX_PCI_CAP_ID_PM);
    /* Next Item Pointer: PCI-X */
    PCIDevSetByte( pPciDev, 0xDC + 1,                      0xE4);
    /* Power Management Capabilities: PM disabled, DSI */
    PCIDevSetWord( pPciDev, 0xDC + 2,
                    0x0002 | VBOX_PCI_PM_CAP_DSI);
    /* Power Management Control / Status Register: PM disabled */
    PCIDevSetWord( pPciDev, 0xDC + 4,                    0x0000);
    /* PMCSR_BSE Bridge Support Extensions: Not supported */
    PCIDevSetByte( pPciDev, 0xDC + 6,                      0x00);
    /* Data Register: PM disabled, always 0 */
    PCIDevSetByte( pPciDev, 0xDC + 7,                      0x00);

    /* PCI-X Configuration Registers *****************************************/
    /* Capability ID: PCI-X Configuration Registers */
    PCIDevSetByte( pPciDev, 0xE4,          VBOX_PCI_CAP_ID_PCIX);
    // TODO MSI
#if 0
    PCIDevSetByte( pPciDev, 0xE4 + 1,                      0x80);
#else
    /* Next Item Pointer: None (Message Signalled Interrupts are disabled) */
    PCIDevSetByte( pPciDev, 0xE4 + 1,                      0x00);
#endif
    /* PCI-X Command: Enable Relaxed Ordering */
    PCIDevSetWord( pPciDev, 0xE4 + 2,        VBOX_PCI_X_CMD_ERO);
    /* PCI-X Status: 32-bit, 66MHz*/
    /** @todo is this value really correct? fff8 doesn't look like actual PCI address */
    PCIDevSetDWord(pPciDev, 0xE4 + 4,                0x0040FFF8);
}

/**
 * @interface_method_impl{PDMDEVREG,pfnConstruct}
 */
static DECLCALLBACK(int) ptnetR3Construct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg)
{
    PPTNETST pThis = PDMINS_2_DATA(pDevIns, PTNETST*);
    int       rc;
    PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);

    /*
     * Initialize the instance data (state).
     * Note! Caller has initialized it to ZERO already.
     */
    RTStrPrintf(pThis->szPrf, sizeof(pThis->szPrf), "PTNET#%d", iInstance);
    PtnetLog(("%s: %s starts\n", pThis->szPrf, __func__));
    pThis->hEvent = NIL_RTSEMEVENT;
    pThis->pDevInsR3    = pDevIns;
    pThis->pDevInsR0    = PDMDEVINS_2_R0PTR(pDevIns);

    /* Interfaces */
    pThis->IBase.pfnQueryInterface          = ptnetR3QueryInterface;

    pThis->INetworkDown.pfnWaitReceiveAvail = ptnetR3NetworkDown_WaitReceiveAvail;
    pThis->INetworkDown.pfnReceive          = ptnetR3NetworkDown_Receive;
    pThis->INetworkDown.pfnXmitPending      = ptnetR3NetworkDown_XmitPending;

    pThis->INetworkConfig.pfnGetMac         = ptnetR3GetMac;
    pThis->INetworkConfig.pfnGetLinkState   = ptnetR3GetLinkState;
    pThis->INetworkConfig.pfnSetLinkState   = ptnetR3SetLinkState;

    pThis->fR0Enabled    = true;

    /* Get config params */
    rc = CFGMR3QueryBytes(pCfg, "MAC", pThis->macConfigured.au8, sizeof(pThis->macConfigured.au8));
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to get MAC address"));
    rc = CFGMR3QueryBool(pCfg, "CableConnected", &pThis->fCableConnected);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to get the value of 'CableConnected'"));

    rc = CFGMR3QueryBoolDef(pCfg, "R0Enabled", &pThis->fR0Enabled, true);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to get the value of 'R0Enabled'"));

    PtnetLog(("%s R0=%s\n", pThis->szPrf,
         pThis->fR0Enabled ? "enabled" : "disabled"));

    /* Initialize critical sections. We do our own locking. */
    rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    AssertRCReturn(rc, rc);

    rc = PDMDevHlpCritSectInit(pDevIns, &pThis->cs, RT_SRC_POS, "PTNET#%d", iInstance);
    if (RT_FAILURE(rc))
        return rc;

    /* Saved state registration. */
    rc = PDMDevHlpSSMRegisterEx(pDevIns, 1, sizeof(PTNETST), NULL,
                                NULL,        ptnetLiveExec, NULL,
                                ptnetSavePrep, ptnetSaveExec, NULL,
                                ptnetLoadPrep, ptnetLoadExec, ptnetLoadDone);
    if (RT_FAILURE(rc))
        return rc;

    /* Set PCI config registers and register ourselves with the PCI bus. */
    ptnetConfigurePciDev(&pThis->pciDevice);
    rc = PDMDevHlpPCIRegister(pDevIns, &pThis->pciDevice);
    if (RT_FAILURE(rc))
        return rc;

#if 0
    /* MSI */
    PDMMSIREG MsiReg;
    RT_ZERO(MsiReg);
    MsiReg.cMsiVectors    = 1;
    MsiReg.iMsiCapOffset  = 0x80;
    MsiReg.iMsiNextOffset = 0x0;
    MsiReg.fMsi64bit      = false;
    rc = PDMDevHlpPCIRegisterMsi(pDevIns, &MsiReg);
    AssertRCReturn(rc, rc);
#endif

    /* Map our registers to IO space (region 0, see ptnetConfigurePciDev)*/
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, PTNETMAP_IO_PCI_BAR,
                                      /* size */PTNET_IO_MASK + 1,
                                      PCI_ADDRESS_SPACE_IO, ptnetMap);
    if (RT_FAILURE(rc))
        return rc;
    /* Map memory for MSI-X (region 2, see ptnetConfigurePciDev) */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, PTNETMAP_MSIX_PCI_BAR,
                                      /* size */ 4096, PCI_ADDRESS_SPACE_MEM,
                                      ptnetMap);
    if (RT_FAILURE(rc))
        return rc;

    /* Create transmit queue */
    rc = PDMDevHlpQueueCreate(pDevIns, sizeof(PDMQUEUEITEMCORE), 1, 0,
                              ptnetTxQueueConsumer, true, "PTNET-Xmit", &pThis->pTxQueueR3);
    if (RT_FAILURE(rc))
        return rc;
    pThis->pTxQueueR0 = PDMQueueR0Ptr(pThis->pTxQueueR3);

    /* Create the RX notifier signaller. */
    rc = PDMDevHlpQueueCreate(pDevIns, sizeof(PDMQUEUEITEMCORE), 1, 0,
                              ptnetCanRxQueueConsumer, true, "PTNET-Rcv", &pThis->pCanRxQueueR3);
    if (RT_FAILURE(rc))
        return rc;
    pThis->pCanRxQueueR0 = PDMQueueR0Ptr(pThis->pCanRxQueueR3);

    /* Register the info item */
    char szTmp[20];
    RTStrPrintf(szTmp, sizeof(szTmp), "ptnet%d", iInstance);
    PDMDevHlpDBGFInfoRegister(pDevIns, szTmp, "PTNET info.", ptnetInfo);

    /* Status driver */
    PPDMIBASE pBase;
    rc = PDMDevHlpDriverAttach(pDevIns, PDM_STATUS_LUN, &pThis->IBase, &pBase, "Status Port");
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the status LUN"));

    /* Network driver */
    rc = PDMDevHlpDriverAttach(pDevIns, 0, &pThis->IBase, &pThis->pDrvBase, "Network Port");
    if (RT_SUCCESS(rc))
    {
        pThis->pDrvR3 = PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMINETWORKUP);
        AssertMsgReturn(pThis->pDrvR3, ("Failed to obtain the PDMINETWORKUP interface!\n"), VERR_PDM_MISSING_INTERFACE_BELOW);
        pThis->pDrvR0 = PDMIBASER0_QUERY_INTERFACE(PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMIBASER0), PDMINETWORKUP);
    }
    else if (   rc == VERR_PDM_NO_ATTACHED_DRIVER
             || rc == VERR_PDM_CFG_MISSING_DRIVER_NAME)
    {
        /* No error! */
        PtnetLog(("%s: %s This adapter is not attached to any network\n", pThis->szPrf, __func__));
    }
    else
        return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the network LUN"));

    rc = RTSemEventCreate(&pThis->hEvent);
    if (RT_FAILURE(rc))
        return rc;

    PtnetLog(("%s: %s ends\n", pThis->szPrf, __func__));

    return VINF_SUCCESS;
}

/**
 * The device registration structure.
 */
const PDMDEVREG g_DeviceNetmapPtNet =
{
    /* Structure version. PDM_DEVREG_VERSION defines the current version. */
    PDM_DEVREG_VERSION,
    /* Device name. */
    "ptnet",
    /* Name of guest context module (no path).
     * Only evalutated if PDM_DEVREG_FLAGS_RC is set (which is not). */
    "VBoxDDRC.rc",
    /* Name of ring-0 module (no path).
     * Only evalutated if PDM_DEVREG_FLAGS_RC is set. */
    "VBoxDDR0.r0",
    /* The description of the device. The UTF-8 string pointed to shall, like this structure,
     * remain unchanged from registration till VM destruction. */
    "Netmap network passthrough device.\n",

    /* Flags, combination of the PDM_DEVREG_FLAGS_* \#defines. */
    PDM_DEVREG_FLAGS_DEFAULT_BITS | /* PDM_DEVREG_FLAGS_RC remove | */ PDM_DEVREG_FLAGS_R0,
    /* Device class(es), combination of the PDM_DEVREG_CLASS_* \#defines. */
    PDM_DEVREG_CLASS_NETWORK,
    /* Maximum number of instances (per VM). */
    ~0U,
    /* Size of the instance data. */
    sizeof(PTNETST),

    /* pfnConstruct */
    ptnetR3Construct,
    /* pfnDestruct */
    ptnetR3Destruct,
    /* pfnRelocate */
    ptnetR3Relocate,
    /* pfnMemSetup */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    ptnetR3Reset,
    /* pfnSuspend */
    ptnetR3Suspend,
    /* pfnResume */
    NULL,
    /* pfnAttach */
    ptnetR3Attach,
    /* pfnDeatch */
    ptnetR3Detach,
    /* pfnQueryInterface */
    NULL,
    /* pfnInitComplete */
    NULL,
    /* pfnPowerOff */
    ptnetR3PowerOff,
    /* pfnSoftReset */
    NULL,

    /* u32VersionEnd */
    PDM_DEVREG_VERSION
};

#endif /* IN_RING3 */
#endif /* !VBOX_DEVICE_STRUCT_TESTCASE */
