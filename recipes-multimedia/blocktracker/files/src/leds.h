#ifndef LED_H
#define LED_H

#include <map>
#include <fstream>

class CLED {   
public:
   CLED(const std::string& str_path, const std::string& str_prefix, unsigned int un_index) {
      for(const std::string& str_color : {"red", "green", "blue"}) {
         std::string strBrightnessPath(str_path);
         strBrightnessPath += (str_prefix + "[" + std::to_string(un_index) + "]:" + str_color + "/");
         strBrightnessPath += "brightness";
         /* Add the mapping */
         m_mapBrightnessPaths[str_color] = strBrightnessPath;
      }
   }          
       
   void SetRed(uint8_t un_red) {
      std::ofstream cBrightnessFile(m_mapBrightnessPaths["red"].c_str());
      cBrightnessFile << static_cast<int>(un_red);    
   }
      
   void SetGreen(uint8_t un_green) {
      std::ofstream cBrightnessFile(m_mapBrightnessPaths["green"].c_str());
      cBrightnessFile << static_cast<int>(un_green);
   }
      
   void SetBlue(uint8_t un_blue) {
      std::ofstream cBrightnessFile(m_mapBrightnessPaths["blue"].c_str());
      cBrightnessFile << static_cast<int>(un_blue);
   }
     
private:
   std::map<std::string, std::string> m_mapBrightnessPaths;
};

#endif
