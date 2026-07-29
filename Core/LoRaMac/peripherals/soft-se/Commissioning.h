#ifndef __COMMISSIONING_H__
#define __COMMISSIONING_H__

#ifdef __cplusplus
extern "C" {
#endif

#define COMMISSIONING_STATIC_DEVICE_EUI 1
#define COMMISSIONING_DEVICE_EUI        { 0x39, 0xdd, 0x74, 0x90, 0x6c, 0x01, 0x3a, 0x52 } //39dd74906c013a52
#define COMMISSIONING_JOIN_EUI          { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define COMMISSIONING_APP_KEY           { 0x94, 0x6F, 0x04, 0xB1, 0xCC, 0xA9, 0x1F, 0x9F, \
						            0x6B, 0x5A, 0x49, 0x83, 0xDD, 0x8E, 0x64, 0xB1 }
/* For LoRaWAN 1.0.x keep this equal to APP_KEY.
 * For LoRaWAN 1.1 set this to the device NwkKey from your network server. */
#define COMMISSIONING_NWK_KEY           COMMISSIONING_APP_KEY

#ifdef __cplusplus
}
#endif

#endif /* __COMMISSIONING_H__ */
