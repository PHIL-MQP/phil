#include <ntcore.h>
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

int main() {
  auto inst = nt::NetworkTableInstance::GetDefault();

  inst.StartClient("localhost", 1735);

  std::cout << "Please start outline viewer in server mode.\n";

  auto table = inst.GetTable("PhilTable");

  std::cout << "Inserting the key value pair [NewKey:goodbye], check outline viewer\n";

  constexpr auto key = "NewKey";
  constexpr auto value = "goodbye";
  table->PutString(key, value);

  std::cout << "Retreiving that new key...\n";

  while (true) {
    auto entry = table->GetEntry(key);
    if (entry.Exists()) {
      auto result = entry.GetString("not found");
      assert(result == value);
      std::cout << result << std::endl;
      break;
    }
  }

  return 0;
}
