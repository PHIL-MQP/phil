#include <ntcore.h>
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

int main() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient("localhost", 1735);

  while (true) {
    auto entry = inst.GetEntry("/Value");
    if (entry.Exists()) {
      std::cout << entry.GetString("not found") << std::endl;
      break;
    }
  }
  return 0;
}
