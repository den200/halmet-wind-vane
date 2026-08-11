#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>
namespace sensesp {
template <class IN, class OUT>
class Transform {
 public:
  Transform(const String& config_path = "") {}
  virtual ~Transform() {}
  virtual void set(const IN&) {}
  virtual bool to_json(JsonObject&) { return true; }
  virtual bool from_json(const JsonObject&) { return true; }
  void load() {}
  void emit(const OUT& v) { if (on_emit) on_emit(v); }
  std::function<void(OUT)> on_emit;   // test hook
};
}  // namespace sensesp
