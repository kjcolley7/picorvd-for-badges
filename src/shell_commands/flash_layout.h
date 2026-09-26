#pragma once

// Where the factory data lives in the probe's own flash, top down:
//
//   [ firmware ... ]  free  [ log segment 2 ][ factory image ][ log segment 1 ]
//                           384KB             20KB             128KB, top of flash
//
// The log started as segment 1 alone (512 records); segment 2 was added
// below the image later, so probes keep their existing log and image
// across the upgrade. Records 0-511 are in segment 1, 512-2047 in
// segment 2. The firmware (~150KB) must end below segment 2 (~1.5MB).

#include <hardware/flash.h>
#include "factory_image.h"

#define FLOG_SEG1_SIZE      (128 * 1024)
#define FLOG_SEG1_OFFSET    (PICO_FLASH_SIZE_BYTES - FLOG_SEG1_SIZE)

#define FIMG_DATA_SECTORS   ((FACTORY_IMAGE_MAX + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE)
#define FIMG_REGION_SIZE    ((1 + FIMG_DATA_SECTORS) * FLASH_SECTOR_SIZE)
#define FIMG_REGION_OFFSET  (FLOG_SEG1_OFFSET - FIMG_REGION_SIZE)
#define FIMG_DATA_OFFSET    (FIMG_REGION_OFFSET + FLASH_SECTOR_SIZE)

#define FLOG_SEG2_SIZE      (384 * 1024)
#define FLOG_SEG2_OFFSET    (FIMG_REGION_OFFSET - FLOG_SEG2_SIZE)

#define FLOG_SEG1_RECORDS   (FLOG_SEG1_SIZE / FLASH_PAGE_SIZE)
#define FLOG_MAX_RECORDS    ((FLOG_SEG1_SIZE + FLOG_SEG2_SIZE) / FLASH_PAGE_SIZE)
