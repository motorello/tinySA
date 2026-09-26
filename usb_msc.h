/*
 * USB mass storage (Bulk-Only Transport) glue between usbcfg.c and usb_msc.c
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 */
#ifndef _USB_MSC_H_
#define _USB_MSC_H_

#define MSC_IF  2     // USB interface number of the mass storage function
#define MSC_EP  3     // Bulk IN (0x83) / OUT (0x03) endpoint number

extern const USBEndpointConfig msc_ep_config;

// Called from usb_event() with the system locked
void msc_usb_reset_I(void);
void msc_configured_I(bool configured);
// Mass storage class requests and CLEAR_FEATURE(ENDPOINT_HALT) of the bulk endpoints
bool msc_requests_hook(USBDriver *usbp);

#endif /* _USB_MSC_H_ */
