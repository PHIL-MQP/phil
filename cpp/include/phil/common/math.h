#pragma once

#include <cstdlib>

#include <eigen3/Eigen/Eigen>

namespace phil {
namespace math {

template<size_t rows, size_t cols>
class Window : public Eigen::Matrix<double, rows, cols> {
 public:
  Window() : head(0), full(false) {}

  /**
   * Over-write the oldest element with new_row
   * @param new_row the new data to insert
   */
  void push(Eigen::Vector3d new_row) {
    Eigen::Matrix<double, rows, cols>::row(head) = new_row;
    ++head;
    if (head >= rows) {
      full = true;
      head = 0;
    }
  }

  bool isFull() {
    return full;
  }

 private:
  size_t head;
  bool full;
};

}
}
