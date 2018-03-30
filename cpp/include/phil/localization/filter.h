#pragma once

#include <memory>

namespace phil {
template<typename T>
class Filter {
 public:
  Filter() : filter(nullptr) {}

  std::unique_ptr<T> filter;

  virtual void ZeroVelocityUpdate() = 0;

};
}
