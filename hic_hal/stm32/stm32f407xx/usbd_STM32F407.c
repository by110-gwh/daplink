/*
 * Copyright (c) 2004-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*----------------------------------------------------------------------------
*      RL-ARM - USB
*----------------------------------------------------------------------------
*      Name:    usbd_STM32F103.c
*      Purpose: Hardware Layer module for ST STM32F103
*      Rev.:    V4.70
*---------------------------------------------------------------------------*/

/* Double Buffering is not supported                                         */

#include <rl_usb.h>
#include "stm32f4xx.h"
#include "IO_Config.h"
#include "cortex_m.h"
#include "string.h"
#include <stdio.h>

#define __NO_USB_LIB_C
#include "usb_config.c"

#define USB_ISTR_W0C_MASK   (ISTR_PMAOVR | ISTR_ERR | ISTR_WKUP | ISTR_SUSP | ISTR_RESET | ISTR_SOF | ISTR_ESOF)
#define VAL_MASK            0xFFFF
#define VAL_SHIFT           16 
#define EP_NUM_MASK         0xFFFF
#define EP_NUM_SHIFT        0

#define USB_DBL_BUF_EP      0x0000
#define USB_DEVICE     ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_DFIFO(i)   *(__IO uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))


UART_HandleTypeDef huart1;
uint32_t gint_flag = 0;
uint32_t rx_state = 0;
uint32_t rx_copy = 0;

/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */

#ifdef __RTX
void __svc(1) USBD_IntrEna(void);
void __SVC_1(void)
{
#else
void          USBD_IntrEna(void)
{
#endif
    HAL_NVIC_SetPriority(OTG_HS_EP1_OUT_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
    HAL_NVIC_SetPriority(OTG_HS_EP1_IN_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *    Return Value:    None
 */
void USBD_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
    GPIOB->MODER |= 2U << GPIO_MODER_MODE14_Pos | 2U << GPIO_MODER_MODE15_Pos;
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15);
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR14 | GPIO_PUPDR_PUPDR15);
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH6| GPIO_AFRH_AFRH7);
    GPIOB->AFR[1] |= 0xCU << GPIO_AFRH_AFSEL14_Pos | 0xCU << GPIO_AFRH_AFSEL15_Pos;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
    USB_OTG_HS->GAHBCFG = USB_OTG_GAHBCFG_GINT | 7 << USB_OTG_GAHBCFG_HBSTLEN_Pos;
    USB_OTG_HS->GUSBCFG = 6U << USB_OTG_GUSBCFG_TRDT_Pos | USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL; // 6 for AHB clock 168MHz
    USB_DEVICE->DCFG = 3U << USB_OTG_DCFG_DSPD_Pos;
    USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_USBSUSPM |
        USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_WUIM |
#ifdef __RTX
           ((USBD_RTX_DevTask   != 0) ? USB_OTG_GINTMSK_SOFM    : 0) |
           ((USBD_RTX_DevTask   != 0) ? USB_OTG_GINTMSK_ESUSPM   : 0);
#else
           ((USBD_P_SOF_Event   != 0) ? USB_OTG_GINTMSK_SOFM    : 0) |
           ((USBD_P_SOF_Event   != 0) ? USB_OTG_GINTMSK_ESUSPM   : 0);
#endif
	
	
	USB_OTG_HS->GINTSTS = 0xFFFFFFFF;
    USBD_IntrEna();
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect(BOOL con)
{
    USB_OTG_HS->GCCFG = USB_OTG_GCCFG_NOVBUSSENS | USB_OTG_GCCFG_PWRDWN;
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */
void USBD_Reset(void)
{
	uint32_t i;
	USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_USBRST;
	for (i = 0; i < 5; i++)
		USB_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
	USB_DEVICE->DAINTMSK = 0x10001;
	USB_DEVICE->DOEPMSK = USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_STUPM;
	USB_DEVICE->DIEPMSK = USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM;
	USB_OTG_HS->GRXFSIZ = 0x200;
	USB_OTG_HS->DIEPTXF0_HNPTXFSIZ = 0x40 << 16 | 0x200;
	USB_OTG_HS->DIEPTXF[0] = 0x40 << 16 | 0x240;
	USB_OTG_HS->DIEPTXF[1] = 0x40 << 16 | 0x280;
	USB_OTG_HS->DIEPTXF[2] = 0x40 << 16 | 0x2C0;
	USB_OTG_HS->DIEPTXF[3] = 0x40 << 16 | 0x300;
	USB_OTG_HS->DIEPTXF[4] = 0x40 << 16 | 0x340;
	USB_OUTEP(0)->DOEPTSIZ = 1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos;
}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend(void)
{
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume(void)
{
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp(void)
{
	USB_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG;
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg(BOOL cfg)
{
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */

void USBD_SetAddress(U32 adr, U32 setup)
{
	if (!setup) {
        return;
    }
	USB_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
	USB_DEVICE->DCFG |= ((uint32_t)adr << 4) & USB_OTG_DCFG_DAD;
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure(BOOL cfg)
{
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */
void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
	uint32_t num, mpsize, ep_type;
    num = pEPD->bEndpointAddress & 0x0F;
    mpsize = pEPD->wMaxPacketSize;
	switch (pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK) {
        case USB_ENDPOINT_TYPE_CONTROL:
            ep_type = 0;
            break;

        case USB_ENDPOINT_TYPE_ISOCHRONOUS:
            ep_type = 1;
            break;

        case USB_ENDPOINT_TYPE_BULK:
            ep_type = 2;
            break;

        case USB_ENDPOINT_TYPE_INTERRUPT:
            ep_type = 3;
            break;
    }
	if (pEPD->bEndpointAddress & 0x80) {
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << num);
		USB_INEP(num)->DIEPCTL = (mpsize & USB_OTG_DIEPCTL_MPSIZ) |
                                   ((uint32_t)ep_type << USB_OTG_DIEPCTL_EPTYP_Pos) |
                                   (num << USB_OTG_DIEPCTL_TXFNUM_Pos);
	} else {
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << num) << 16);
		USB_OUTEP(num)->DOEPCTL = (mpsize & USB_OTG_DOEPCTL_MPSIZ) |
                                   ((uint32_t)ep_type << USB_OTG_DOEPCTL_EPTYP_Pos);
		USB_OUTEP(num)->DOEPTSIZ = 1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos |
			(USB_OUTEP(num)->DOEPCTL & USB_OTG_DOEPCTL_MPSIZ) << USB_OTG_DOEPTSIZ_XFRSIZ_Pos;
		USB_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
	}
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP(U32 dir)
{
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP(U32 EPNum)
{
	if (EPNum & 0x80) {
		USB_INEP(EPNum & 0x0F)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;
	} else {
		USB_OUTEP(EPNum & 0x0F)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;
	}
}


/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP(U32 EPNum)
{
	if (EPNum & 0x80) {
		USB_INEP(EPNum & 0x0F)->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;
	} else {
		USB_OUTEP(EPNum & 0x0F)->DOEPCTL &= ~USB_OTG_DOEPCTL_USBAEP;
	}
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP(U32 EPNum)
{
	USBD_ClearEPBuf(EPNum);
	if (EPNum & 0x80) {
		USB_INEP(EPNum & 0x0F)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
	} else {
		USB_OUTEP(EPNum & 0x0F)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
	}
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP(U32 EPNum)
{
	if (EPNum & 0x80) {
		USB_INEP(EPNum & 0xF)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	} else 
		USB_OUTEP(EPNum & 0xF)->DOEPCTL |= USB_OTG_DIEPCTL_STALL;
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP(U32 EPNum)
{
	if (EPNum & 0x80) {
		USB_INEP(EPNum & 0xF)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
	} else {
		USB_OUTEP(EPNum & 0xF)->DOEPCTL &= ~USB_OTG_DIEPCTL_STALL;
	}
	USBD_ResetEP(EPNum);
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf(U32 EPNum)
{
	EPNum &= 0x0F;
	USB_OTG_HS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (EPNum << 6));
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
}


/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

U32 USBD_ReadEP(U32 EPNum, U8 *pData, U32 bufsz)
{
	
	uint32_t count32b;
	uint32_t i;
	if (((rx_state & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos) < bufsz)
		bufsz = ((rx_copy & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos);
	count32b = ((uint32_t)bufsz + 3U) / 4U;
	for (i = 0U; i < count32b; i++) {
		((__packed uint32_t *)pData)[i] = USB_DFIFO(0U);
	}
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
	return bufsz;
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

U32 USBD_WriteEP(U32 EPNum, U8 *pData, U32 cnt)
{
	uint32_t len32b;
	uint32_t i;
	EPNum &= 0x0F;
	USB_INEP(EPNum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
	USB_INEP(EPNum)->DIEPTSIZ |= 1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos;
	USB_INEP(EPNum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
	USB_INEP(EPNum)->DIEPTSIZ |= cnt;
	USB_INEP(EPNum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	if (cnt != 0) {
		len32b = (cnt + 3U) / 4U;
		while (USB_INEP(EPNum)->DTXFSTS < len32b);
		for (i = 0U; i < len32b; i++) {
			USB_DFIFO(EPNum) = ((uint32_t*)pData)[i];
		}	
	}
	return cnt;
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

U32 USBD_GetFrame(void)
{
	return (USB_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
}


#ifdef __RTX
U32 LastError;                          /* Last Error                         */

/*
 *  Get USB Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

U32 USBD_GetError(void)
{
    return (LastError);
}
#endif



void USBD_Handler(void)
{
    uint32_t ep_intr;
    uint32_t epnum;
    uint32_t epint;
    if (gint_flag & USB_OTG_GINTSTS_SOF) {
		
		#ifdef __RTX
        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_SOF;

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_SOF_Event) {
            USBD_P_SOF_Event();
        }

        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_SOF;
#endif
    }
    
    
    if (gint_flag & USB_OTG_GINTSTS_ESUSP) {
        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_ESUSP;
    }
    
    if (gint_flag & USB_OTG_GINTSTS_WKUINT) {
		USBD_Resume();
#ifdef __RTX
        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_WKUINT;

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_RESUME,  USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Resume_Event) {
            USBD_P_Resume_Event();
        }

        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_WKUINT;
#endif
	}
	
    if (gint_flag & USB_OTG_GINTSTS_USBSUSP) {
		if (USBD_P_Suspend_Event) {
			USBD_P_Suspend_Event();
		}
#ifdef __RTX
        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Suspend_Event) {
            USBD_P_Suspend_Event();
        }

        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;
#endif
    }
    
    if (gint_flag & USB_OTG_GINTSTS_RXFLVL) {
        rx_state = USB_OTG_HS->GRXSTSP;
		epnum = rx_state & USB_OTG_GRXSTSP_EPNUM;
        if (((rx_state & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) ==  2U)
        {
            if ((rx_state & USB_OTG_GRXSTSP_BCNT) != 0U)
            {
				USB_OTG_HS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
				rx_copy = rx_state;
				
#ifdef __RTX

                if (USBD_RTX_EPTask[n]) {       /* OUT Packet                         */
                    isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[n]);
                }

#else

                if (USBD_P_EP[epnum]) {
                    USBD_P_EP[epnum](USBD_EVT_OUT);
                }

#endif
            }
        }
        else if (((rx_state & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) ==  6U)
        {
			USB_OTG_HS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
			rx_copy = rx_state;
#ifdef __RTX

			if (USBD_RTX_EPTask[n]) {       /* SETUP Packet                       */
				isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[n]);
			}

#else

			if (USBD_P_EP[epnum]) {
				USBD_P_EP[epnum](USBD_EVT_SETUP);
			}

#endif
        }
        
    }
    
    if (gint_flag & USB_OTG_GINTSTS_USBRST) {
		USBD_Reset();
        usbd_reset_core();
#ifdef __RTX

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
        }
		
#else
		
		if (USBD_P_Reset_Event) {
			USBD_P_Reset_Event();
		}

#endif

    }
    
    if (gint_flag & USB_OTG_GINTSTS_ENUMDNE) {
        USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
        USB_INEP(0)->DIEPTSIZ = 64 << USB_OTG_DIEPCTL_MPSIZ_Pos;
        USB_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;
    }

    if (gint_flag & USB_OTG_GINTMSK_OEPINT) {
        epnum = 0U;
        ep_intr = USB_DEVICE->DAINT;
        ep_intr &= USB_DEVICE->DAINTMSK;
        ep_intr >>= 16;
        while (ep_intr != 0U) {
            if ((ep_intr & 0x1U) != 0U) {
                epint = USB_OUTEP((uint32_t)epnum)->DOEPINT;
                epint &= USB_DEVICE->DOEPMSK;
                if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
                    USB_OUTEP(epnum)->DOEPINT = (USB_OTG_DOEPINT_STUP);
					USB_OUTEP(epnum)->DOEPTSIZ = 1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos;
					USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
                }
				if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
                    USB_OUTEP(epnum)->DOEPINT = (USB_OTG_DOEPINT_XFRC);
					USB_OUTEP(epnum)->DOEPTSIZ = 1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos;
					USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;	
				}
            }
            ep_intr >>= 1U;
            epnum++;
        }
    }
    
	
    if (gint_flag & USB_OTG_GINTMSK_IEPINT) {
        epnum = 0U;
        ep_intr = USB_DEVICE->DAINT;
        ep_intr &= USB_DEVICE->DAINTMSK;
		while (ep_intr != 0U) {
            if ((ep_intr & 0x1U) != 0U) {
                epint = USB_INEP((uint32_t)epnum)->DIEPINT;
                epint &= USB_DEVICE->DIEPMSK;
				if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
					USB_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_XFRC;
#ifdef __RTX

					if (USBD_RTX_EPTask[n]) {       /* IN Packet                          */
						isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[n]);
					}

#else

					if (USBD_P_EP[epnum]) {
						USBD_P_EP[epnum](USBD_EVT_IN);
					}

#endif
				}
			}
            ep_intr >>= 1U;
            epnum++;
		}
	}
	
    HAL_NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
    HAL_NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void OTG_HS_EP1_OUT_IRQHandler(void)
{
    gint_flag = USB_OTG_HS->GINTSTS;
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
	USBD_SignalHandler();
}

void OTG_HS_EP1_IN_IRQHandler(void)
{
    gint_flag = USB_OTG_HS->GINTSTS;
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
	USBD_SignalHandler();
}

void OTG_HS_IRQHandler(void)
{
    gint_flag = USB_OTG_HS->GINTSTS;
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
	USBD_SignalHandler();
}
