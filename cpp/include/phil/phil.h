#pragma  once

namespace phil {

class Phil {

 public:
  int GetPosition();

  void GiveEncoder(int *encoder);

  int *encoder_;
};

}
