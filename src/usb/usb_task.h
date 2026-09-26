#ifndef __USB_TASK_H
#define __USB_TASK_H

extern volatile uint32_t g_usb_heartbeat;

void vTaskUsb(void *pvParams);

#endif // __USB_TASK_H
