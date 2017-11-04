#include <phil/phil.h>

void phil::Phil::GiveEncoder(int *encoder) {
  encoder_ = encoder;
}

int phil::Phil::GetPosition() {
  return 0;
}
