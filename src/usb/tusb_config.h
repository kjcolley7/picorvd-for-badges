#ifndef __TUSB_CONFIG_H
#define __TUSB_CONFIG_H

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_CDC             2
#define CFG_TUD_CDC_RX_BUFSIZE  (256)
#define CFG_TUD_CDC_TX_BUFSIZE  (256)

#define CFG_TUD_CDC_EP_BUFSIZE   64

// Mass storage: the drive the factory image is uploaded through (msc_disk.cpp).
#define CFG_TUD_MSC             1
#define CFG_TUD_MSC_EP_BUFSIZE  512

#include <tusb_option.h>

#ifdef __cplusplus
extern "C" {
#endif

void usbd_serial_init(void);

// The flash-unique-id serial string set by usbd_serial_init(), which doubles
// as this probe's identity in the factory log.
const char *usbd_serial_str(void);

#ifdef __cplusplus
}
#endif

#endif // __TUSB_CONFIG_H
