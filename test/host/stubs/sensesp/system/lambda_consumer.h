#pragma once
#include <functional>
namespace sensesp {
template <class T>
class LambdaConsumer {
 public:
  LambdaConsumer(std::function<void(T)> f) : f_(f) {}
  void set(const T& v) { f_(v); }
 private:
  std::function<void(T)> f_;
};
}  // namespace sensesp
