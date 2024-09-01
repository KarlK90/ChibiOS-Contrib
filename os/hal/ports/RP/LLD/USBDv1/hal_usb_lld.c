/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Get endpoint control register.
 */
#define EP_CTRL(ep)       (usb_dpram->ep_ctrl[ep - 1])
/**
 * @brief   Get buffer control register for endpoint.
 */
#define BUF_CTRL(ep)      (usb_dpram->ep_buf_ctrl[ep])

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (RP_USB_USE_USBD0 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static struct {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Buffer mode for isochronous in buffer control register.
 */
static uint16_t usb_isochronous_buffer_mode(uint16_t size) {
  switch (size) {
    case 128:
      return USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_VALUE_128;
    case 256:
      return USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_VALUE_256;
    case 512:
      return USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_VALUE_512;
    case 1024:
      return USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_VALUE_1024;
    default:
      return USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_VALUE_128;
  }
}

/**
 * @brief   Calculate isochronous buffer size in valid step.
 * @details Valid buffer size is one of 128, 256, 512 or 1024.
 */
static uint16_t usb_isochronous_buffer_size(uint16_t max_size) {
  uint16_t size;

  /* Double buffer offset must be one of 128, 256, 512 or 1024. */
  size = ((max_size - 1) / 128 + 1) * 128;
  if (size == 384) {
    size = 512;
  } else if (size == 640 || size == 768 || size > 1024) {
    size = 1024;
  }
  return size;
}

/**
 * @brief   Calculate next offset for buffer data, 64 bytes aligned.
 */
static uint16_t usb_buffer_next_offset(USBDriver *usbp, uint16_t size, bool is_double) {
  uint32_t offset;

  offset = usbp->noffset;
  usbp->noffset += is_double ? size * 2 : size;

  return offset;
}

/**
 * @brief   Reset endpoint 0.
 */
static void reset_ep0(USBDriver *usbp) {
  usbp->epc[0]->out_state->next_pid = 1U;
  usbp->epc[0]->in_state->next_pid = 1U;
}

/**
 * @brief   Prepare buffer for receiving data.
 */
uint32_t usb_prepare_out_ep_buffer(USBDriver *usbp, usbep_t ep, uint8_t buffer_index) {
    uint32_t buf_ctrl = 0;
    const USBEndpointConfig *epcp = usbp->epc[ep];
    USBOutEndpointState *oesp = usbp->epc[ep]->out_state;

    /* PID */
    buf_ctrl |= oesp->next_pid ? USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_PID_0_BITS : 0;
    oesp->next_pid ^= 1U;

    uint16_t buf_len = oesp->rxsize < epcp->out_maxsize ? oesp->rxsize : epcp->out_maxsize;
    buf_ctrl |= USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_AVAILABLE_0_BITS | buf_len;
    buf_ctrl &= ~USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_FULL_0_BITS;

    if (oesp->rxcnt + buf_len >= oesp->rxsize) {
        /* Last buffer */
        buf_ctrl |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS;
    }

    if (buffer_index) {
      buf_ctrl = buf_ctrl << 16;
    }

    return buf_ctrl;
}

/**
 * @brief   Prepare for receiving data from host.
 */
static void usb_prepare_out_ep(USBDriver *usbp, usbep_t ep) {
  uint32_t buf_ctrl;
  uint32_t ep_ctrl;

  if (ep == 0) {
    ep_ctrl = usb_hw->sie_ctrl;
  } else {
    ep_ctrl = EP_CTRL(ep).out;
  }

  /* Fill first buffer */
  buf_ctrl = usb_prepare_out_ep_buffer(usbp, ep, 0);

  /* To avoid short packet, we use single buffered here. */
  /* Single buffered */
  ep_ctrl &= ~(USB_SIE_CTRL_EP0_DOUBLE_BUF_BITS | USB_SIE_CTRL_EP0_INT_2BUF_BITS);
  ep_ctrl |= USB_SIE_CTRL_EP0_INT_1BUF_BITS;

  if (ep == 0) {
    usb_hw->sie_ctrl = ep_ctrl;
  } else {
    EP_CTRL(ep).out = ep_ctrl;
  }

  BUF_CTRL(ep).out = buf_ctrl;
}

/**
 * @brief   Prepare buffer for sending data.
 */
static uint32_t usb_prepare_in_ep_buffer(USBDriver *usbp, usbep_t ep, uint8_t buffer_index) {
    uint8_t *buff;
    uint16_t buf_len;
    uint32_t buf_ctrl = 0;
    const USBEndpointConfig *epcp = usbp->epc[ep];
    USBInEndpointState *iesp = usbp->epc[ep]->in_state;

    /* txsize - txlast gives size of data to be sent but not yet in the buffer */
    buf_len = epcp->in_maxsize < iesp->txsize - iesp->txlast ?
              epcp->in_maxsize : iesp->txsize - iesp->txlast;

    iesp->txlast += buf_len;

    /* Host only? */
    if (iesp->txsize <= iesp->txlast) {
      /* Last buffer */
      buf_ctrl |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS;
    }

    /* PID */
    buf_ctrl |= iesp->next_pid ? USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_PID_0_BITS : 0;
    iesp->next_pid ^= 1U;

    /* Copy data into hardware buffer */
    buff = (uint8_t*)iesp->hw_buf + (buffer_index == 0 ? 0 : iesp->buf_size);
    memcpy((void *)buff, (void *)iesp->txbuf, buf_len);
    iesp->txbuf += buf_len;

    buf_ctrl |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_0_BITS |
                USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS |
                buf_len;

    if (buffer_index) {
      buf_ctrl = buf_ctrl << 16;
    }

    return buf_ctrl;
}

/**
 * @brief   Prepare endpoint for sending data.
 */
static void usb_prepare_in_ep(USBDriver *usbp, usbep_t ep) {
  uint32_t buf_ctrl;
  uint32_t ep_ctrl;
  USBInEndpointState *iesp = usbp->epc[ep]->in_state;

  if (ep == 0) {
    ep_ctrl = usb_hw->sie_ctrl;
  } else {
    ep_ctrl = EP_CTRL(ep).in;
  }

  /* Fill first buffer */
  buf_ctrl = usb_prepare_in_ep_buffer(usbp, ep, 0);

  /* Second buffer if required */
  /* iesp->txsize - iesp->txlast gives size even not in buffer */
  if (iesp->txsize - iesp->txlast > 0) {
    buf_ctrl |= usb_prepare_in_ep_buffer(usbp, ep, 1);
  }

  if (buf_ctrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_1_BITS) {
    /* Double buffered */
    ep_ctrl &= ~USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    ep_ctrl |= USB_SIE_CTRL_EP0_DOUBLE_BUF_BITS | USB_SIE_CTRL_EP0_INT_2BUF_BITS;
  } else {
    /* Single buffered */
    ep_ctrl &= ~(USB_SIE_CTRL_EP0_DOUBLE_BUF_BITS | USB_SIE_CTRL_EP0_INT_2BUF_BITS);
    ep_ctrl |= USB_SIE_CTRL_EP0_INT_1BUF_BITS;
  }

  if (ep == 0) {
    usb_hw->sie_ctrl = ep_ctrl;
  } else {
    EP_CTRL(ep).in = ep_ctrl;
  }

  BUF_CTRL(ep).in = buf_ctrl;
}

/**
 * @brief   Work on an endpoint after transfer.
 */
static void usb_serve_endpoint(USBDriver *usbp, usbep_t ep, bool is_in) {
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *oesp;
  USBInEndpointState *iesp;
  uint16_t n;

  if (is_in) {
    /* IN endpoint */
    iesp = usbp->epc[ep]->in_state;

    /* txlast is size sent + size of last buffer */
    iesp->txcnt = iesp->txlast;
    n = iesp->txsize - iesp->txcnt;
    if (n > 0) {
      /* Transfer not completed, there are more packets to send. */
      usb_prepare_in_ep(usbp, ep);
    } else {
      /* Transfer complete */
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  } else {
    /* OUT endpoint */
    oesp = usbp->epc[ep]->out_state;

    /* Length received */
    n = BUF_CTRL(ep).out & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_LENGTH_0_BITS;

    /* Copy received data into user buffer */
    memcpy((void *)oesp->rxbuf, (void *)oesp->hw_buf, n);
    oesp->rxbuf += n;
    oesp->rxcnt += n;
    oesp->rxsize -= n;

    oesp->rxpkts -= 1;

    /* Short packet or all packetes have been received. */
    if (oesp->rxpkts == 0 || n < epcp->out_maxsize) {
      /* Transifer complete */
      _usb_isr_invoke_out_cb(usbp, ep);
    } else {
      /* Receive remained data */
      usb_prepare_out_ep(usbp, ep);
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if RP_USB_USE_USBD0 || defined(__DOXYGEN__)

/**
 * @brief   USB low priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(RP_USBCTRL_IRQ_HANDLER) {
  OSAL_IRQ_PROLOGUE();

  USBDriver *usbp = &USBD1;
  uint32_t ints = usb_hw->ints;

  /* USB setup packet handling. */
  if (ints & USB_INTS_SETUP_REQ_BITS) {
    usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;

    reset_ep0(usbp);

    _usb_isr_invoke_setup_cb(usbp, 0);
  }

  /* USB bus reset condition handling. */
  if (ints & USB_INTS_BUS_RESET_BITS) {
    usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;

    _usb_reset(usbp);
  }

  /* USB bus SUSPEND condition handling.*/
  if (ints & USB_INTS_DEV_SUSPEND_BITS) {
    usb_hw_clear->sie_status = USB_SIE_STATUS_SUSPENDED_BITS;

    _usb_suspend(usbp);
  }

  /* Resume condition handling */
  if (ints & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
    usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;

    _usb_wakeup(usbp);
  }

  /* SOF handling.*/
  if (ints & USB_INTS_DEV_SOF_BITS) {
    /* SOF interrupt was used to detect resume of the USB bus after issuing a
     * remote wake up of the host, therefore we disable it again. */
    if (usbp->config->sof_cb == NULL) {
      usb_hw_clear->inte = USB_INTE_DEV_SOF_BITS;
    }
    if (usbp->state == USB_SUSPENDED) {
      _usb_wakeup(usbp);
    }

    _usb_isr_invoke_sof_cb(usbp);

    /* Clear SOF flag by reading SOF_RD */
    (void)usb_hw->sof_rd;
  }

  /* Endpoint events handling.*/
  if (ints & USB_INTS_BUFF_STATUS_BITS) {
    uint32_t buf_status = usb_hw->buf_status;
    uint32_t bit = 1U;
    for (uint8_t i = 0; buf_status && i < 32; i++) {
      if (buf_status & bit) {
        /* Clear flag */
        usb_hw_clear->buf_status = bit;
        /* Finish on the endpoint or transfer remained data */
        usb_serve_endpoint(&USBD1, i >> 1U, (i & 1U) == 0);

        buf_status &= ~bit;
      }
      bit <<= 1U;
    }
  }

#if RP_USB_USE_ERROR_DATA_SEQ_INTR == TRUE
  if (ints & USB_INTE_ERROR_DATA_SEQ_BITS) {
    usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
  }
#endif /* RP_USB_USE_ERROR_DATA_SEQ_INTR */

  OSAL_IRQ_EPILOGUE();
}

#endif /* RP_USB_USE_USBD0 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
#if RP_USB_USE_USBD0 == TRUE
  /* Driver initialization.*/
  usbObjectInit(&USBD1);

  /* Reset buffer offset. */
  USBD1.noffset = 0;
#endif
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
#if RP_USB_USE_USBD0 == TRUE
  if (&USBD1 == usbp) {
    if (usbp->state == USB_STOP) {
      /* Reset usb controller */
      hal_lld_peripheral_reset(RESETS_RESET_USBCTRL_BITS);
      hal_lld_peripheral_unreset(RESETS_RESET_USBCTRL_BITS);

      /* Clear any previos state in dpram and hw regs */
      memset(usb_hw, 0, sizeof(usb_hw_t));
      memset(usb_dpram, 0, sizeof(usb_device_dpram_t));

      /* Mux the controller to the onboard usb phy */
      usb_hw->muxing = USB_USB_MUXING_SOFTCON_BITS | USB_USB_MUXING_TO_PHY_BITS;

#if RP_USB_FORCE_VBUS_DETECT == TRUE
      /* Force VBUS detect so the device thinks it is plugged into a host */
      usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS | USB_USB_PWR_VBUS_DETECT_BITS;
#else
#if RP_USE_EXTERNAL_VBUS_DETECT == TRUE
      /* If VBUS is detected by pin without USB VBUS DET pin,
       * define usb_vbus_detect which returns true if VBUS is enabled.
       */
      if (usb_vbus_detect()) {
        usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS | USB_USB_PWR_VBUS_DETECT_BITS;
      }
#endif /* RP_USE_EXTERNAL_VBUS_DETECT */
#endif /* RP_USB_FORCE_VBUS_DETECT */

      /* Reset procedure enforced on driver start.*/
      usb_lld_reset(usbp);

      /* Enable the USB controller in device mode. */
      usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

      /* Enable an interrupt per EP0 transaction */
      usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

      /* Enable interrupts */
      usb_hw->inte = USB_INTE_SETUP_REQ_BITS |
                  USB_INTE_DEV_RESUME_FROM_HOST_BITS |
                  USB_INTE_DEV_SUSPEND_BITS |
                  USB_INTE_BUS_RESET_BITS |
                  USB_INTE_BUFF_STATUS_BITS;

      if (usbp->config->sof_cb != NULL) {
        usb_hw->inte |= USB_INTE_DEV_SOF_BITS;
      }

#if RP_USB_USE_ERROR_DATA_SEQ_INTR == TRUE
      usb_hw->inte |= USB_INTE_ERROR_DATA_SEQ_BITS;
#endif /* RP_USB_USE_ERROR_DATA_SEQ_INTR */

      /* Enable USB interrupt. */
      nvicEnableVector(RP_USBCTRL_IRQ_NUMBER, RP_IRQ_USB0_PRIORITY);
    }
  }
#endif
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
#if RP_USB_USE_USBD0 == TRUE
  if (&USBD1 == usbp) {
    if (usbp->state != USB_STOP) {
      /* Disable USB interrupt */
      usb_hw->inte = 0;
      nvicDisableVector(RP_USBCTRL_IRQ_NUMBER);

      /* Disable controller */
      usb_hw_clear->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
    }
  }
#endif
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
    /* EP0 initialization.*/
    usbp->epc[0] = &ep0config;
    usb_lld_init_endpoint(usbp, 0U);

    /* Reset device address. */
    usb_hw->dev_addr_ctrl = 0U;

    /* Reset USB memory */
    usbp->noffset = 0U;

    /* Clear all non control endpoint registers */
    for (int ep = 1; ep < (USB_MAX_ENDPOINTS - 1); ep++) {
        EP_CTRL(ep).in  = USB_DEVICE_DPRAM_EP1_IN_CONTROL_RESET;
        EP_CTRL(ep).out = USB_DEVICE_DPRAM_EP1_OUT_CONTROL_RESET;
    }

    for (int ep = 0; ep < USB_MAX_ENDPOINTS; ep++) {
        BUF_CTRL(ep).in  = USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_RESET;
        BUF_CTRL(ep).out = USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_RESET;
    }
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  /* Set address to hardware here. */
  usb_hw->dev_addr_ctrl = USB_ADDR_ENDP_ADDRESS_BITS & (usbp->address << USB_ADDR_ENDP_ADDRESS_LSB);
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
    uint16_t                 buf_size;
    uint16_t                 buf_offset;
    uint32_t                 buf_ctrl;
    const USBEndpointConfig *epcp = usbp->epc[ep];

    if (ep == 0) {
        epcp->in_state->hw_buf    = (uint8_t *)&usb_dpram->ep0_buf_a;
        epcp->in_state->buf_size  = 64;
        epcp->in_state->next_pid  = 0U;
        epcp->out_state->hw_buf   = (uint8_t *)&usb_dpram->ep0_buf_a;
        epcp->out_state->buf_size = 64;
        epcp->out_state->next_pid = 0U;
        usb_hw_set->sie_ctrl      = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
        return;
    }

    if (epcp->in_state) {
        buf_ctrl                 = 0U;
        BUF_CTRL(ep).in          = buf_ctrl;
        epcp->in_state->next_pid = 0U;

        if (epcp->ep_mode == USB_EP_MODE_TYPE_ISOC) {
            buf_size = usb_isochronous_buffer_size(epcp->in_maxsize);
            buf_ctrl |= usb_isochronous_buffer_mode(buf_size) << USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_LSB;
        } else {
            buf_size = 64;
        }
        buf_offset               = usb_buffer_next_offset(usbp, buf_size, true);
        epcp->in_state->hw_buf   = (uint8_t *)&usb_dpram->epx_data[buf_offset];
        epcp->in_state->buf_size = buf_size;

        EP_CTRL(ep).in  = USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENABLE_BITS | (epcp->ep_mode << USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENDPOINT_TYPE_LSB) | ((uint8_t *)epcp->in_state->hw_buf - (uint8_t *)usb_dpram);
        BUF_CTRL(ep).in = buf_ctrl;
    }

    if (epcp->out_state) {
        buf_ctrl                  = 0U;
        BUF_CTRL(ep).out          = buf_ctrl;
        epcp->out_state->next_pid = 0U;

        if (epcp->ep_mode == USB_EP_MODE_TYPE_ISOC) {
            buf_size = usb_isochronous_buffer_size(epcp->in_maxsize);
            buf_ctrl |= usb_isochronous_buffer_mode(buf_size) << USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_DOUBLE_BUFFER_ISO_OFFSET_LSB;
        } else {
            buf_size = 64;
        }
        buf_offset                = usb_buffer_next_offset(usbp, buf_size, false);
        epcp->out_state->hw_buf   = (uint8_t *)&usb_dpram->epx_data[buf_offset];
        epcp->out_state->buf_size = buf_size;

        EP_CTRL(ep).out  = USB_DEVICE_DPRAM_EP1_OUT_CONTROL_ENABLE_BITS | (epcp->ep_mode << USB_DEVICE_DPRAM_EP1_OUT_CONTROL_ENDPOINT_TYPE_LSB) | ((uint8_t *)epcp->out_state->hw_buf - (uint8_t *)usb_dpram);
        BUF_CTRL(ep).out = buf_ctrl;
    }
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  for (uint8_t ep = 1; ep < (USB_MAX_ENDPOINTS - 1); ep++) {
    EP_CTRL(ep).in &= ~USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENABLE_BITS;
    EP_CTRL(ep).out &= ~USB_DEVICE_DPRAM_EP1_OUT_CONTROL_ENABLE_BITS;
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  if (BUF_CTRL(ep).out & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_STALL_BITS) {
    return EP_STATUS_STALLED;
  }
  if (EP_CTRL(ep).out & USB_DEVICE_DPRAM_EP1_OUT_CONTROL_ENABLE_BITS) {
    return EP_STATUS_ACTIVE;
  }
  return EP_STATUS_DISABLED;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  if (BUF_CTRL(ep).in & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS) {
    return EP_STATUS_STALLED;
  }
  if (EP_CTRL(ep).in & USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENABLE_BITS) {
    return EP_STATUS_ACTIVE;
  }
  return EP_STATUS_DISABLED;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  (void)usbp;
  (void)ep;
  /* Copy data from hardware buffer to user buffer */
  memcpy((void *)buf, (void *)usb_dpram->setup_packet, 8);
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  USBOutEndpointState *oesp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  if (oesp->rxsize == 0U) {
    /* Special case for zero sized packets.*/
    oesp->rxpkts = 1U;
  } else {
    oesp->rxpkts = (uint16_t)((oesp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);
  }

  usb_prepare_out_ep(usbp, ep);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  USBInEndpointState *iesp = usbp->epc[ep]->in_state;
  iesp->txlast = 0;

  /* Prepare IN endpoint. */
  usb_prepare_in_ep(usbp, ep);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
    if (ep == 0) {
        usb_hw_set->ep_stall_arm = USB_EP_STALL_ARM_EP0_OUT_BITS;
    }
    BUF_CTRL(ep).out |= USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_STALL_BITS;
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
    if (ep == 0) {
        usb_hw_set->ep_stall_arm = USB_EP_STALL_ARM_EP0_IN_BITS;
    }
    BUF_CTRL(ep).in |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS;
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
    if (ep > 0) {
        BUF_CTRL(ep).out &= ~USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_STALL_BITS;
    }
    usbp->epc[ep]->out_state->next_pid = 0U;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
    if (ep > 0) {
        BUF_CTRL(ep).in &= ~USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS;
    }
    usbp->epc[ep]->in_state->next_pid = 0U;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
