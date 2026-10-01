#pragma once

// GPS-fed NTP server. The boat's computers (the Odroid plotter in particular)
// have no battery-backed clock, so they boot at a stale date and every TLS
// connection fails until NTP corrects it — and offshore there is no internet
// NTP at all. HALMET is the first thing up on the boat and already sits on the
// N2K backbone, where the GNSS publishes PGN 126992 (System Time) every second,
// so it answers NTP on UDP 123 with that time.
//
// Trust rules, because a wrong clock is worse than none:
// - Only PGN 126992 with a GNSS time source. The AIS on this boat (Raymarine
//   E22158) reports 2007 in PGN 129029 — the GPS week-number rollover — so no
//   other PGN is read.
// - Dates before kMinValidUnix are rejected outright (rollover lands ~19.6
//   years in the past).
// - A time is only taken when two consecutive messages agree with each other
//   to within kAgreeUs, so one corrupt frame can never set the clock.
// - No reply at all when unlocked or stale: the client then falls back to its
//   next server instead of being handed a wrong time.
//
// Everything runs on the ReactESP loop (N2K handler + UDP poll), so the shared
// state needs no locking.

#include <Arduino.h>
#include <N2kMessages.h>
#include <NMEA2000.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "sensesp_app.h"

namespace halmet {

class GpsTimeServer {
 public:
  void begin(tNMEA2000* n2k) {
    n2k->SetMsgHandler(&GpsTimeServer::on_n2k);
    static const unsigned long kRxPGNs[] PROGMEM = {126992L, 0};
    n2k->ExtendReceiveMessages(kRxPGNs);
    // 2 ms poll: the receive timestamp is taken when the packet is read, so
    // this bounds the error that adds (N2K latency itself is tens of ms).
    sensesp::event_loop()->onRepeat(2, []() { instance().poll_udp(); });
  }

  static GpsTimeServer& instance() {
    static GpsTimeServer s;
    return s;
  }

  bool locked() const {
    return have_time_ && (esp_timer_get_time() - lock_rx_us_) < kStaleUs;
  }
  // Seconds since the last accepted GPS time, or -1 if never locked.
  int age_s() const {
    return have_time_ ? (int)((esp_timer_get_time() - lock_rx_us_) / 1000000)
                      : -1;
  }
  uint32_t served() const { return served_; }

 private:
  static constexpr int64_t kMinValidUnix = 1767225600;  // 2026-01-01T00:00Z
  static constexpr int64_t kAgreeUs = 1000000;          // consecutive msgs, 1 s
  // ESP32 crystal is good to ~±50 ppm: 10 min free-running drifts ≤ 30 ms.
  static constexpr int64_t kStaleUs = 10LL * 60 * 1000000;
  static constexpr uint32_t kNtpEpochOffset = 2208988800UL;  // 1900→1970
  static constexpr const char* kTag = "gps-time";

  static void on_n2k(const tN2kMsg& msg) {
    if (msg.PGN == 126992L) instance().on_system_time(msg);
  }

  void on_system_time(const tN2kMsg& msg) {
    unsigned char sid;
    uint16_t days;
    double secs;
    tN2kTimeSource src;
    if (!ParseN2kPGN126992(msg, sid, days, secs, src)) return;
    const int64_t rx_us = esp_timer_get_time();
    if (src != N2ktimes_GPS && src != N2ktimes_GLONASS) {
      log_reject(msg.Source, "time source is not GNSS", (int)src);
      return;
    }
    if (days == N2kUInt16NA || !(secs >= 0.0 && secs < 86400.0)) return;
    const int64_t utc_us =
        (int64_t)days * 86400LL * 1000000LL + (int64_t)(secs * 1e6);
    if (utc_us / 1000000 < kMinValidUnix) {
      log_reject(msg.Source, "date before 2026", days);
      return;
    }

    // Commit only when this message agrees with the previous candidate,
    // carried forward by the local clock.
    const bool agrees =
        cand_valid_ &&
        llabs((cand_utc_us_ + (rx_us - cand_rx_us_)) - utc_us) < kAgreeUs;
    cand_utc_us_ = utc_us;
    cand_rx_us_ = rx_us;
    cand_valid_ = true;
    if (!agrees) return;

    if (!have_time_) {
      ESP_LOGI(kTag, "GPS time locked from N2K source %u: %lld (unix s)",
               msg.Source, (long long)(utc_us / 1000000));
    }
    lock_utc_us_ = utc_us;
    lock_rx_us_ = rx_us;
    have_time_ = true;
  }

  void log_reject(unsigned char source, const char* why, int value) {
    // Once a minute at most; the GNSS repeats every second.
    const int64_t now = esp_timer_get_time();
    if (now - last_reject_log_us_ < 60LL * 1000000) return;
    last_reject_log_us_ = now;
    ESP_LOGW(kTag, "PGN 126992 from source %u ignored: %s (%d)", source, why,
             value);
  }

  int64_t now_utc_us() const {
    return lock_utc_us_ + (esp_timer_get_time() - lock_rx_us_);
  }

  static void put_ts(uint8_t* p, int64_t utc_us) {
    const uint32_t sec = (uint32_t)(utc_us / 1000000) + kNtpEpochOffset;
    const uint32_t frac =
        (uint32_t)(((uint64_t)(utc_us % 1000000) << 32) / 1000000);
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(sec >> (24 - 8 * i));
    for (int i = 0; i < 4; i++) p[4 + i] = (uint8_t)(frac >> (24 - 8 * i));
  }

  void poll_udp() {
    if (!udp_open_) {
      // Bind once the WiFi stack is up; the socket survives reconnects.
      if (WiFi.status() != WL_CONNECTED) return;
      udp_open_ = udp_.begin(123);
      if (udp_open_) ESP_LOGI(kTag, "NTP server listening on UDP 123");
      return;
    }
    if (udp_.parsePacket() <= 0) return;
    const int64_t rx_us = esp_timer_get_time();
    uint8_t req[48];
    const int n = udp_.read(req, sizeof(req));
    udp_.clear();
    if (n < 48) return;
    const uint8_t mode = req[0] & 0x07;
    if (mode != 3 || !locked()) return;  // client requests only; silent if unlocked

    uint8_t res[48] = {0};
    const uint8_t vn = (req[0] >> 3) & 0x07;
    res[0] = (0 << 6) | (vn << 3) | 4;  // LI none, client's version, server
    res[1] = 1;                          // stratum 1: GNSS reference
    res[2] = req[2];                     // poll interval: echo the client's
    res[3] = (uint8_t)-6;                // precision 2^-6 s ≈ 16 ms
    // Root dispersion 50 ms (16.16 fixed point): honest for N2K-relayed time.
    const uint32_t disp = (uint32_t)(0.05 * 65536);
    for (int i = 0; i < 4; i++) res[8 + i] = (uint8_t)(disp >> (24 - 8 * i));
    memcpy(&res[12], "GPS", 4);  // reference id
    put_ts(&res[16], lock_utc_us_);                            // reference
    memcpy(&res[24], &req[40], 8);                             // originate
    put_ts(&res[32], lock_utc_us_ + (rx_us - lock_rx_us_));    // receive
    put_ts(&res[40], now_utc_us());                            // transmit

    udp_.beginPacket(udp_.remoteIP(), udp_.remotePort());
    udp_.write(res, sizeof(res));
    udp_.endPacket();
    served_++;
  }

  WiFiUDP udp_;
  bool udp_open_ = false;
  bool cand_valid_ = false;
  int64_t cand_utc_us_ = 0, cand_rx_us_ = 0;
  bool have_time_ = false;
  int64_t lock_utc_us_ = 0, lock_rx_us_ = 0;
  int64_t last_reject_log_us_ = -60LL * 1000000;
  uint32_t served_ = 0;
};

}  // namespace halmet
