#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <vector>
#include <list>
#include <cmath>
#include <cstdint>

#include <signal.h>
#include <unistd.h>


#include "packet_control_interface.h"

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

/****************************************/
/****************************************/

volatile bool bShutdownSignal = false;

void InteruptSignalHandler(int n_unused) {
   bShutdownSignal = true;
}

/****************************************/
/****************************************/

int main(int n_arg_count, char* ppch_args[]) {
   /* Register the SIGINT handler to shut down system cleanly on abort */
   ::signal(SIGINT, InteruptSignalHandler);

   enum class EActuatorInputLimit : uint8_t {
      LAUTO = 0, L100 = 1, L150 = 2, L500 = 3, L900 = 4
   };

   std::vector<CRangeFinder> vecRangeFinders;
   std::vector<CLED> vecLEDs;

   CPacketControlInterface* m_pcPowerManagementInterface =
      new CPacketControlInterface("power management", "/dev/ttySC0", 57600);

   if(!m_pcPowerManagementInterface->Open()) {
         std::cerr << "Error" << std::endl << "Could not open the device" << std::endl;
         return -ENODEV;
   }

   /* Override actuator input limit to 100mA */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            static_cast<const uint8_t>(EActuatorInputLimit::L100));

   /* Enable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, true);   

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

   while(!bShutdownSignal) {
      std::vector<std::list<uint16_t>> vecReadings(NUM_DEVICES);

      for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_DEVICES; un_dev_idx++) {
         vecReadings[un_dev_idx].push_front(vecRangeFinders[un_dev_idx].Read());
         if(vecReadings[un_dev_idx].size() > 5) {
            vecReadings[un_dev_idx].pop_back();
         }

         std::list<uint16_t> lstReadings;
         std::copy(std::begin(vecReadings[un_dev_idx]),
                   std::end(vecReadings[un_dev_idx]),
                   std::begin(lstReadings));

         lstReadings.sort();
         
         std::list<uint16_t>::iterator itMedianReading = std::begin(lstReadings);
         std::advance(itMedianReading, static_cast<size_t>(lstReadings.size() / 2.0f));

         float fAverageRaw = static_cast<float>(*itMedianReading);
         
         /*
         for(uint16_t un_reading : vecReadings[un_dev_idx]) {
            fAverageRaw += static_cast<float>(un_reading);
         }
         fAverageRaw /= static_cast<float>(vecReadings[un_dev_idx].size());
         */

         float fAverage = 67.0f * std::pow(fAverageRaw + 1, -0.4f);
       
         vecLEDs[un_dev_idx].SetRed(std::floor(35.0f - std::fmin(35.0f, std::fabs(0 - fAverage))));
         vecLEDs[un_dev_idx].SetGreen(std::floor(35.0f - std::fmin(35.0f, std::fabs(35 - fAverage))));
         vecLEDs[un_dev_idx].SetBlue(std::floor(35.0f - std::fmin(35.0f, std::fabs(70 - fAverage))));
      }
   }

   /* Disable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, false);
   /* Disable actuator input limit override */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            static_cast<const uint8_t>(EActuatorInputLimit::LAUTO));

}


