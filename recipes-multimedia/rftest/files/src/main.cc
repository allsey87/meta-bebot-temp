#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <vector>
#include <cmath>

#include <unistd.h>

#define TEST_DELAY 2000

#define NUM_DEVICES 12u
#define NUM_LEDS NUM_DEVICES
#define NUM_RFS NUM_DEVICES

struct SColor {
   uint8_t Red, Green, Blue;
};

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

class CLED {   
   public:
      CLED(const std::string& str_path, const std::string& str_prefix, unsigned int un_index) {
         for(const std::string& str_color : {"red", "green", "blue"}) {
            std::string strBrightnessPath(str_path);
            strBrightnessPath += (str_prefix + "[" + std::to_string(un_index) + "]:" + str_color + "/");
            strBrightnessPath += "brightness";
               
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

class CLEDDeck {
public:
   CLEDDeck() {
      for(unsigned int un_led_idx = 0; un_led_idx < NUM_LEDS; un_led_idx++) {
         m_vecLEDs.emplace_back("/sys/class/leds/", "pca963x:led_deck", un_led_idx + 1);
      }
   }
   
   void SetColor(unsigned int un_index, const SColor& s_color) {
      CLED& cLED = m_vecLEDs[un_index];
      cLED.SetRed(s_color.Red);
      cLED.SetGreen(s_color.Green);
      cLED.SetBlue(s_color.Blue);
   }
   
   void SetColor(unsigned int un_index, uint8_t un_red, uint8_t un_green, uint8_t un_blue) {
      CLED& cLED = m_vecLEDs[un_index];
      cLED.SetRed(un_red);
      cLED.SetGreen(un_green);
      cLED.SetBlue(un_blue);
   }
   
   void SetAllColors(const SColor& s_color) {
      for(CLED& cLED : m_vecLEDs) {
         cLED.SetRed(s_color.Red);
         cLED.SetGreen(s_color.Green);
         cLED.SetBlue(s_color.Blue);
      }
   }
   
   void SetAllColors(uint8_t un_red, uint8_t un_green, uint8_t un_blue) {
      for(CLED& cLED : m_vecLEDs) {
         cLED.SetRed(un_red);
         cLED.SetGreen(un_green);
         cLED.SetBlue(un_blue);
      }
   }

private:
   std::vector<CLED> m_vecLEDs;
};

int main() {
   std::vector<CRangeFinder> vecRangeFinders;
   std::vector<CLED> vecLEDs;

   for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_DEVICES; un_dev_idx++) {
      vecRangeFinders.emplace_back("/sys/class/rfs/", "bebot:rf_chassis", un_dev_idx);
      vecLEDs.emplace_back("/sys/class/leds/", "pca963x:led_deck", un_dev_idx);
   }

   /*
   for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_DEVICES; un_dev_idx++) {
      for(int j = 0;  j < 128; j++) {
         vecLEDs[un_dev_idx].SetRed(j);
         usleep(TEST_DELAY);
      }
      for(int j = 128;  j > 0; j--) {
         vecLEDs[un_dev_idx].SetRed(j);
         usleep(TEST_DELAY);
      }
      for(int j = 0;  j < 128; j++) {
         vecLEDs[un_dev_idx].SetGreen(j);
         usleep(TEST_DELAY);
      }
      for(int j = 128;  j > 0; j--) {
         vecLEDs[un_dev_idx].SetGreen(j);
         usleep(TEST_DELAY);
      }for(int j = 0;  j < 128; j++) {
         vecLEDs[un_dev_idx].SetBlue(j);
         usleep(TEST_DELAY);
      }
      for(int j = 128;  j > 0; j--) {
         vecLEDs[un_dev_idx].SetBlue(j);
         usleep(TEST_DELAY);
      }
   }
   */

   for(;;) {
      for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_DEVICES; un_dev_idx++) {
         float fAverageRaw = (
            vecRangeFinders[un_dev_idx].Read() +
            vecRangeFinders[un_dev_idx].Read() +
            vecRangeFinders[un_dev_idx].Read()) / 3.0f;

         float fAverage = 67.0f * std::pow(fAverageRaw + 1, -0.4f);
       
         vecLEDs[un_dev_idx].SetRed(std::floor(35.0f - std::fmin(35.0f, std::fabs(0 - fAverage))));
         vecLEDs[un_dev_idx].SetGreen(std::floor(35.0f - std::fmin(35.0f, std::fabs(35 - fAverage))));
         vecLEDs[un_dev_idx].SetBlue(std::floor(35.0f - std::fmin(35.0f, std::fabs(70 - fAverage))));
      }
   }
}


