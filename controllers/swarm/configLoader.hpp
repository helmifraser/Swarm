#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

class configLoader {
public:
  std::map<std::string, std::string> LoadConfig(std::string filename);
};
