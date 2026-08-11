#pragma once
#include <map>
#include <string>
// Minimal stand-in: enough of the JsonObject surface for to_json/from_json.
class JsonObject {
 public:
  struct Proxy {
    JsonObject* o; std::string k;
    template <class T> bool is() const {
      auto it = o->m_.find(k);
      return it != o->m_.end() && it->second.second == want<T>();
    }
    void operator=(float v)  { o->m_[k] = {(double)v, 'f'}; }
    void operator=(double v) { o->m_[k] = {v, 'f'}; }
    void operator=(int v)    { o->m_[k] = {(double)v, 'i'}; }
    operator float() const { return (float)o->m_.at(k).first; }
    operator int() const   { return (int)o->m_.at(k).first; }
   private:
    template <class T> static char want();
  };
  Proxy operator[](const char* k) { return Proxy{this, k}; }
  Proxy operator[](const char* k) const {
    return Proxy{const_cast<JsonObject*>(this), k};
  }
  std::map<std::string, std::pair<double, char>> m_;
};
template <> inline char JsonObject::Proxy::want<float>() { return 'f'; }
template <> inline char JsonObject::Proxy::want<int>()   { return 'i'; }
