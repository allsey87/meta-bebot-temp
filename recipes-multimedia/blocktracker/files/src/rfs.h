#ifndef RFS_H
#define RFS_H

#include <map>
#include <fstream>

class CRangeFinder {
public:
   CRangeFinder(const std::string& str_path, const std::string& str_prefix, unsigned int un_index) {
      m_strRfBrightnessPath = str_path;
      m_strRfBrightnessPath += (str_prefix + "[" + std::to_string(un_index) + "]/");
      m_strRfBrightnessPath += "brightness";
   }

   uint16_t Read() {
      uint16_t unValue;
      std::ifstream cBrightnessFile(m_strRfBrightnessPath.c_str());
      cBrightnessFile >> unValue;
      return unValue;
   }
public:
   std::string m_strRfBrightnessPath;
};

#endif
