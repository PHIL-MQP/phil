#include <cstdlib>
#include <unistd.h>
#include <random>
#include <ntcore.h>
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <phil/common/common.h>

int main() {
  auto inst = nt::NetworkTableInstance::GetDefault();

  inst.StartClient("localhost", NT_DEFAULT_PORT);
  usleep(500000);

  std::cout << "Please start outline viewer in server mode.\n";

  auto table = inst.GetTable(phil::kTableName);

  std::cout << "Inserting the key value pair [NewKey:goodbye], check outline viewer\n";

  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_int_distribution<int> uniform_dist(1, 100);
  int rand = uniform_dist(e1);
  std::cout << "setting value " << rand << "\n";
  table->PutNumberArray(phil::kPoseKey, rand);

  std::cout << "Retreiving that new key...\n";

  while (true) {
    auto entry = table->GetEntry(phil::kPoseKey);
    if (entry.Exists()) {
      auto result = entry.GetDoubleArray(-1.0);
      for (auto d : result) {
        std::cout << d << ", ";
      }
      std::cout << "\n";
      break;
    }
  }

  std::cout << "success\n";
  usleep(5000);

  return 0;
}
