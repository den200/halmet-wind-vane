#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "sensesp/system/saveable.h"

namespace sensesp {

// Read-only "help card" for the SensESP web UI. It carries no editable
// settings — its ConfigItem title + description render as a card in the config
// page, which lets us ship wiring/power/calibration instructions inside the
// device itself (useful offline on a boat). The empty schema means the card
// shows just the explanatory text, no input fields.
class HelpNote : public FileSystemSaveable {
 public:
  explicit HelpNote(const String& config_path)
      : FileSystemSaveable(config_path) {}
  bool to_json(JsonObject& root) override { return true; }
  bool from_json(const JsonObject& root) override { return true; }
};

inline const String ConfigSchema(const HelpNote&) {
  // The web UI treats an empty "properties" object as "still loading" and shows
  // a blank placeholder card. We add a single property whose key starts with
  // "_": that makes properties non-empty (card renders), but the frontend
  // filters "_"-prefixed keys out of the form, so no input field is shown. The
  // help text itself is the ConfigItem description, rendered by the UI as HTML.
  return "{\"type\":\"object\",\"properties\":{\"_\":{\"type\":\"string\"}}}";
}
inline const bool ConfigRequiresRestart(const HelpNote&) { return false; }

}  // namespace sensesp
