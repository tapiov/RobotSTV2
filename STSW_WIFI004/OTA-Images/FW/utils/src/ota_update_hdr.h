/* Here we define the struct ota_update_hdr */

#ifndef __OTA_UPDATE_HDR_H
#define __OTA_UPDATE_HDR_H

#define MAX_STR_LEN 32
#define G_UPDATE_COOKIE (0x00228800)
#define G_BASE (0x0)
#define G_FINAL_HEXSPEAK (0x8505E11A)

struct ota_update_hdr {
  uint32_t id;   /* set to UPDATE_COOKIE to say we're ready */
  uint32_t hdrlen;  /* Length of update header, LE */
  uint32_t base; /* Offset into main flash to write, LE */
  uint32_t len;  /* Length of data to write, LE */
  uint8_t  model[MAX_STR_LEN];
  uint8_t  version[MAX_STR_LEN];
  uint8_t  hash[32];  /* Used to check the integrity of the firmware in clear */
  uint8_t  iv[12];    /* 96-bit Initialization Vector (IV) for GCM operation */
  uint8_t  tag[16];   /* Used to check the authenticity of the firmware during GCM */
  uint8_t  data[0];   /* Data follows, padded to uint32_t */
};

#endif /* __OTA_UPDATE_HDR_H */
