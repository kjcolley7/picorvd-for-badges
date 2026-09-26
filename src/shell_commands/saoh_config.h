#ifndef SAOH_CONFIG_H
#define SAOH_CONFIG_H

// The RP2040's I2C block NAKs the final byte of every read, so it cannot size a
// transfer from the first received byte. Leaving this off makes saoh_core use
// fixed/maximum-length reads it sizes itself.
#define SAOH_CFG_SUPPORT_SMBUS_BLKREAD 0

#define SAOH_CFG_LOG_MIN_LEVEL LOGL_DEBUG
#define SAOH_CFG_CMD_RETRY_CNT 3

#endif
