#pragma once

// The PICORVD USB mass-storage drive (msc_disk.cpp).

#ifdef __cplusplus
extern "C" {
#endif

// Build the volume in RAM from the stored factory image and log. Call once,
// before tusb_init().
void msc_disk_init(void);

// Watches for uploads and stores them; see msc_disk.cpp.
void vTaskMscDisk(void *pvParams);

#ifdef __cplusplus
}
#endif
