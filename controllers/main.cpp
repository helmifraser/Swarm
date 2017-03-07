#include "configLoader.hpp"

int main(int argc, char const *argv[]) {
  configLoader *fileLoader = new configLoader();
  std::string filename = "test-config.ini";

  std::map<std::string, std::string> map = fileLoader->LoadConfig(filename);

  std::cout << map["PS_THRESHOLD"] << '\n';
  std::cout << map["var1"] << '\n';
  return 0;
}
