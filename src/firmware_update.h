#pragma once
// Firmware update from a phone or laptop browser, no cable and no PlatformIO:
//
//   GET  /update        one small page: running version, "check latest
//                       release" (GitHub API, from the browser), file picker,
//                       upload progress, reboot wait.
//   GET  /api/firmware  {"version","built","slot"} — the page polls this after
//                       the reboot to confirm the new image is running.
//   POST /api/firmware  raw firmware.bin body → written to the other OTA app
//                       slot → verified → reboot.
//
// Why a raw body and not a multipart form: esp_http_server has no multipart
// parser, and a browser can POST a File object as the body directly, so the
// parser is simply not needed.
//
// Integrity: the image the build produces carries an appended SHA-256, and
// Update.end() → esp_ota_set_boot_partition() → esp_image_verify() checks it
// and the segment checksums before the new slot is ever marked bootable. A
// truncated or corrupted download therefore fails here, not at boot. On top of
// that the first bytes are checked for the ESP32 image magic and chip id, so a
// .bin built for another chip is refused before anything is erased. The core
// also has bootloader rollback enabled: an image that does not reach
// initArduino() is rolled back to the previous slot on the next reset.

#include <Arduino.h>
#include <Update.h>
#include <esp_app_format.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_ota_ops.h>

#include <algorithm>
#include <memory>

#include "sensesp/net/http_server.h"
#include "sensesp_app.h"
#include "version.h"

namespace halmet {

// SensESPApp keeps its HTTPServer in a protected member with no getter. Naming
// the member through a derived type is the well-defined way to reach it: the
// access check happens on forming the pointer-to-member, which a derived class
// may do, and applying it to the base object is ordinary member access.
struct HttpServerAccess : sensesp::SensESPApp {
  static std::shared_ptr<sensesp::HTTPServer>& of(sensesp::SensESPApp& app) {
    return app.*(&HttpServerAccess::http_server_);
  }
};

static const char kUpdatePage[] = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HALMET Wind — firmware update</title>
<style>
body{font:16px -apple-system,system-ui,sans-serif;max-width:34em;margin:1.5em auto;padding:0 1em;color:#222}
h2{margin:.2em 0 .6em}button{font:inherit;padding:.55em 1em;border-radius:6px;border:1px solid #888;background:#f4f4f4}
button:disabled{opacity:.5}progress{width:100%;height:1em}.m{color:#666}.ok{color:#1a7f37}.err{color:#b42318}
code{background:#eee;padding:.1em .3em;border-radius:3px}ol{padding-left:1.3em}li{margin:.3em 0}
</style>
<h2>HALMET Wind — firmware update</h2>
<p>Running <b id="v">…</b> <span class="m" id="b"></span></p>
<p><button id="chk">Check latest release</button> <span id="rel"></span></p>
<ol class="m">
<li>Download the <code>.bin</code> from the release. If the boat Wi-Fi has no internet, do it on mobile data first — it lands in Files.</li>
<li>Back on the boat Wi-Fi, pick that file below and press Flash. The board reboots by itself when done; wind data pauses for about ten seconds.</li>
</ol>
<p><input type="file" id="f" accept=".bin,application/octet-stream"></p>
<p><button id="go" disabled>Flash</button></p>
<progress id="p" max="100" value="0" hidden></progress>
<p id="s"></p>
<script>
const $=id=>document.getElementById(id);
const REPO='den200/halmet-wind-vane';
let running='';
function info(){return fetch('/api/firmware',{cache:'no-store'}).then(r=>r.json());}
info().then(j=>{running=j.version;$('v').textContent='v'+j.version;$('b').textContent='built '+j.built+', slot '+j.slot;}).catch(()=>{});
$('chk').onclick=()=>{
  $('rel').textContent='checking…';
  fetch('https://api.github.com/repos/'+REPO+'/releases/latest').then(r=>r.json()).then(j=>{
    const tag=(j.tag_name||'').replace(/^v/,'');
    if(!tag){$('rel').textContent='no release found';return;}
    const bins=(j.assets||[]).filter(a=>a.name.endsWith('.bin'));
    let html=(tag===running?'<span class="ok">v'+tag+' — you are up to date</span>':'<b>v'+tag+'</b> available (running v'+running+')');
    bins.forEach(a=>{html+=' · <a href="'+a.browser_download_url+'">'+a.name+'</a>';});
    $('rel').innerHTML=html;
  }).catch(()=>{$('rel').innerHTML='<span class="err">GitHub not reachable from this network — download on mobile data, then come back.</span>';});
};
$('f').onchange=()=>{$('go').disabled=!$('f').files.length;$('s').textContent='';};
$('go').onclick=()=>{
  const file=$('f').files[0];if(!file)return;
  if(!confirm('Flash '+file.name+' ('+(file.size/1024|0)+' KB) to the HALMET?'))return;
  $('go').disabled=true;$('chk').disabled=true;$('f').disabled=true;
  $('p').hidden=false;$('p').value=0;$('s').textContent='Uploading…';
  const x=new XMLHttpRequest();
  x.open('POST','/api/firmware');
  x.setRequestHeader('Content-Type','application/octet-stream');
  x.upload.onprogress=e=>{if(e.lengthComputable)$('p').value=e.loaded/e.total*100;};
  x.onerror=()=>{$('s').innerHTML='<span class="err">Upload failed (connection lost).</span>';$('f').disabled=false;$('go').disabled=false;};
  x.onload=()=>{
    if(x.status!==200){$('s').innerHTML='<span class="err">Rejected: '+x.responseText+'</span>';$('f').disabled=false;$('go').disabled=false;$('chk').disabled=false;return;}
    $('s').textContent='Written and verified. Rebooting — waiting for the board…';
    let tries=0;
    const poll=()=>{info().then(j=>{
      if(j.version!==running||tries>4){$('s').innerHTML='<span class="ok">Running v'+j.version+' (built '+j.built+', slot '+j.slot+').</span>';setTimeout(()=>location.reload(),1500);}
      else{tries++;setTimeout(poll,2000);}
    }).catch(()=>{tries++;setTimeout(poll,2000);});};
    setTimeout(poll,5000);
  };
  x.send(file);
};
</script>
)HTML";

static const char* kUpdTag = "fw-update";

// One 4 KB receive buffer on the heap: the HTTP server task runs on a 6 KB
// stack, and Update.write() does the flash erase/program work underneath.
static constexpr size_t kChunk = 4096;
static constexpr int kRecvRetries = 3;

// Answer, then return ESP_FAIL so esp_http_server closes the socket: a body
// refused half-way must not be re-parsed as the next request.
static esp_err_t reject(httpd_req_t* req, const char* status, const char* msg) {
  ESP_LOGE(kUpdTag, "refused: %s", msg);
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, msg);
  return ESP_FAIL;
}

static esp_err_t handle_firmware_post(httpd_req_t* req) {
  const size_t total = req->content_len;
  if (total < sizeof(esp_image_header_t) + 1024) {
    return reject(req, "400 Bad Request", "body too small to be a firmware image");
  }
  std::unique_ptr<uint8_t[]> buf(new uint8_t[kChunk]);
  // begin() also rejects an image larger than the free OTA slot.
  if (!Update.begin(total, U_FLASH)) {
    return reject(req, "500 Internal Server Error", Update.errorString());
  }
  ESP_LOGW(kUpdTag, "flashing %u bytes into %s", (unsigned)total,
           esp_ota_get_next_update_partition(nullptr)->label);

  size_t received = 0;
  int retries = 0;
  while (received < total) {
    const size_t want = std::min(kChunk, total - received);
    const int n = httpd_req_recv(req, reinterpret_cast<char*>(buf.get()), want);
    if (n == HTTPD_SOCK_ERR_TIMEOUT && ++retries <= kRecvRetries) continue;
    if (n <= 0) {
      Update.abort();
      return reject(req, "400 Bad Request", "connection lost during upload");
    }
    retries = 0;
    if (received == 0) {
      // Refuse the obviously-wrong file before the first byte is written.
      if (static_cast<size_t>(n) < sizeof(esp_image_header_t)) {
        Update.abort();
        return reject(req, "400 Bad Request", "first packet too short");
      }
      const auto* hdr = reinterpret_cast<const esp_image_header_t*>(buf.get());
      if (hdr->magic != ESP_IMAGE_HEADER_MAGIC) {
        Update.abort();
        return reject(req, "400 Bad Request",
                      "not an ESP32 firmware image (bad magic byte)");
      }
      if (hdr->chip_id != ESP_CHIP_ID_ESP32) {
        Update.abort();
        return reject(req, "400 Bad Request",
                      "image is built for a different chip, not the ESP32");
      }
    }
    if (Update.write(buf.get(), n) != static_cast<size_t>(n)) {
      const char* why = Update.errorString();
      Update.abort();
      return reject(req, "500 Internal Server Error", why);
    }
    received += n;
  }

  // end(true) verifies the whole image (checksums + appended SHA-256) and only
  // then points the bootloader at the new slot.
  if (!Update.end(true)) {
    return reject(req, "500 Internal Server Error", Update.errorString());
  }
  ESP_LOGW(kUpdTag, "update verified, rebooting");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  // Same pattern SensESP uses for its own /api/device/restart.
  sensesp::event_loop()->onDelay(1000, []() { ESP.restart(); });
  return ESP_OK;
}

static esp_err_t handle_firmware_get(httpd_req_t* req) {
  char json[160];
  snprintf(json, sizeof(json),
           "{\"version\":\"%s\",\"built\":\"%s %s\",\"slot\":\"%s\"}",
           FW_VERSION, __DATE__, __TIME__,
           esp_ota_get_running_partition()->label);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

static esp_err_t handle_update_page(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, kUpdatePage, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// Call after the SensESP app exists (builder.get_app()). The handlers go on
// SensESP's own server, so they sit behind the same web-UI authentication if
// one is configured there.
inline void add_firmware_update_handlers() {
  using sensesp::HTTPRequestHandler;
  auto& server = HttpServerAccess::of(*sensesp::sensesp_app);
  auto page = std::make_shared<HTTPRequestHandler>(1 << HTTP_GET, "/update",
                                                   handle_update_page);
  auto info = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/api/firmware", handle_firmware_get);
  auto flash = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_POST, "/api/firmware", handle_firmware_post);
  server->add_handler(page);
  server->add_handler(info);
  server->add_handler(flash);
}

}  // namespace halmet
