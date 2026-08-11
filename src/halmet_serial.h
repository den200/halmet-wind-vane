#ifndef HALMET_SRC_HALMET_SERIAL_H_
#define HALMET_SRC_HALMET_SERIAL_H_

#include <esp_mac.h>

#include <cstdint>

// inline: this is a definition in a header. With one translation unit it links
// today, but any second .cpp that includes it would be a duplicate symbol.
//
// The N2K device NAME is built from this, so it has to be stable across boots:
// zero-initialise, and leave the id at 0 rather than reading uninitialised
// stack if the efuse read ever fails.
inline uint64_t GetBoardSerialNumber() {
  uint8_t chipid[6] = {0, 0, 0, 0, 0, 0};
  if (esp_efuse_mac_get_default(chipid) != ESP_OK) {
    return 0;
  }
  return ((uint64_t)chipid[0] << 0) + ((uint64_t)chipid[1] << 8) +
         ((uint64_t)chipid[2] << 16) + ((uint64_t)chipid[3] << 24) +
         ((uint64_t)chipid[4] << 32) + ((uint64_t)chipid[5] << 40);
}

#endif  // HALMET_SRC_HALMET_SERIAL_H_
