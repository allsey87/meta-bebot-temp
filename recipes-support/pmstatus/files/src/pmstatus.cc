/**
 * @file pmstatus.cc
 * @brief Print the status of the BeBot power management to the screen
 * @author: Michael Allwright
 */

#include <iostream>

#include "packet_control_interface.h"

/****************************************/
/****************************************/

int main(int n_arg_count, char* ppch_args[]) {
   CPacketControlInterface* pcPowerManagementInterface =
      new CPacketControlInterface("power management", "/dev/ttySC0", 57600);
      
   if(!pcPowerManagementInterface->Open()) {
         std::cerr << "Error" << std::endl << "Could not open the device" << std::endl;
         return -ENODEV;
   }
   
   pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_UPTIME);
   pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_PM_STATUS);
   pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL);
   pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_USB_STATUS);
   
   /* wait for responses from the manipulator interface */
   bool bWaitingForUptimeResponse = true;
   bool bWaitingForPmStatusResponse = true;
   bool bWaitingForBattLvlResponse = true;
   bool bWaitingForUsbStatusResponse = true;
   
   while(bWaitingForUptimeResponse || bWaitingForPmStatusResponse || bWaitingForBattLvlResponse || bWaitingForUsbStatusResponse) {
      pcPowerManagementInterface->ProcessInput();
      if(pcPowerManagementInterface->GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
         const CPacketControlInterface::CPacket& cPacket = pcPowerManagementInterface->GetPacket();
         switch(cPacket.GetType()) {
         case CPacketControlInterface::CPacket::EType::GET_UPTIME:
            if(cPacket.GetDataLength() == 4) {
               const uint8_t* punPacketData = cPacket.GetDataPointer();
               uint32_t unUptime = 
                  (punPacketData[0] << 24) |
                  (punPacketData[1] << 16) |
                  (punPacketData[2] << 8)  |
                  (punPacketData[3] << 0);
               std::cerr << "Uptime: " << unUptime << std::endl;
            }
            bWaitingForUptimeResponse = false;
            break;
         case CPacketControlInterface::CPacket::EType::GET_BATT_LVL:
            if(cPacket.GetDataLength() == 2) {
               std::cerr << "System battery: "
                         << 17u * cPacket.GetDataPointer()[0]
                         << "mV" << std::endl;
               std::cerr << "Actuator battery: "
                         << 17u * cPacket.GetDataPointer()[1]
                         << "mV" << std::endl;
            }
            bWaitingForBattLvlResponse = false;       
            break;
         case CPacketControlInterface::CPacket::EType::GET_USB_STATUS:
            if(cPacket.GetDataLength() == 4) {
               const uint8_t* punPacketData = cPacket.GetDataPointer();
               std::cerr << "USB: " << ((punPacketData[0] == 0) ? "Disabled" : "Enabled") << std::endl;
               std::cerr << "High Speed Mode: " << ((punPacketData[1] == 0) ? "Inactive" : "Active") << std::endl;
               std::cerr << "Suspend Mode: " << ((punPacketData[2] == 0) ? "Inactive" : "Active") << std::endl;
               std::cerr << "USB Charger Type: ";
               switch(punPacketData[3]) {
               case 0:
                  std::cerr << "DISABLED";
                  break;
               case 1:
                  std::cerr << "WAIT";
                  break;
               case 2:
                  std::cerr << "DCP";
                  break;
               case 3:
                  std::cerr << "CDP";
                  break;
               case 4:
                  std::cerr << "SDP";
                  break;
               case 5:
                  std::cerr << "SE1L";
                  break;
               case 6:
                  std::cerr << "SE1H";
                  break;
               case 7:
                  std::cerr << "SE1S";
                  break;
               default:
                  std::cerr << "ERROR";
                  break;
               }
            }
            std::cerr << std::endl;
            bWaitingForUsbStatusResponse = false;
            break;
         case CPacketControlInterface::CPacket::EType::GET_PM_STATUS:
            if(cPacket.GetDataLength() == 9) {
               const uint8_t* punPacketData = cPacket.GetDataPointer();
               std::cerr << "System Power: " << ((punPacketData[0] == 0) ? "Off" : "On") << std::endl;
               std::cerr << "Actuator Power: " << ((punPacketData[1] == 0) ? "Off" : "On") << std::endl;
               std::cerr << "Passthrough Power: " << ((punPacketData[2] == 0) ? "Off" : "On") << std::endl;
               std::cerr << "System Battery: " << ((punPacketData[3] == 0) ? "Not Charging" : "Charging") << std::endl;
               std::cerr << "Actuator Battery: " << ((punPacketData[4] == 0) ? "Not Charging" : "Charging") << std::endl;
               std::cerr << "System Input Limit: ";
               switch(punPacketData[5]) {
               case 0:
                  std::cerr << "0";
                  break;
               case 1:
                  std::cerr << "100";
                  break;
               case 2:
                  std::cerr << "150";
                  break;
               case 3:
                  std::cerr << "500";
                  break;
               case 4:
                  std::cerr << "800";
                  break;
               case 5:
                  std::cerr << "900";
                  break;
               case 6:
                  std::cerr << "1500";
                  break;
               case 7:
                  std::cerr << "2500";
                  break;
               default:
                  std::cerr << "ERROR";
                  break;
               }
               std::cerr << std::endl;
               std::cerr << "Actuator Input Limit: ";
               switch(punPacketData[6]) {
               case 0:
                  std::cerr << "100";
                  break;
               case 1:
                  std::cerr << "150";
                  break;
               case 2:
                  std::cerr << "500";
                  break;
               case 3:
                  std::cerr << "900";
                  break;
               case 4:
                  std::cerr << "1500";
                  break;
               case 5:
                  std::cerr << "2000";
                  break;
               case 6:
                  std::cerr << "EXT";
                  break;
               case 7:
                  std::cerr << "PTM";
                  break;
               case 8:
                  std::cerr << "HIZ";
                  break;
               default:
                  std::cerr << "ERROR";
                  break;
               }
               std::cerr << std::endl;
               std::cerr << "Adapter Input State: ";
               switch(punPacketData[7]) {
               case 0:
                  std::cerr << "NORMAL";
                  break;
               case 1:
                  std::cerr << "OVER_VOLTAGE";
                  break;
               case 2:
                  std::cerr << "WEAK_SOURCE";
                  break;
               case 3:
                  std::cerr << "UNDER_VOLTAGE";
                  break;
               default:
                  std::cerr << "ERROR";
                  break;
               }
               std::cerr << std::endl;
               std::cerr << "USB Input State: ";
               switch(punPacketData[8]) {
               case 0:
                  std::cerr << "NORMAL";
                  break;
               case 1:
                  std::cerr << "OVER_VOLTAGE";
                  break;
               case 2:
                  std::cerr << "WEAK_SOURCE";
                  break;
               case 3:
                  std::cerr << "UNDER_VOLTAGE";
                  break;
               default:
                  std::cerr << "ERROR";
                  break;
               }
               std::cerr << std::endl;
               bWaitingForPmStatusResponse = false;
            }
            break;
         default:
            std::cerr << "WARNING: Unrecognised Packet Recieved" << std::endl;
         }
      }
   }
   return 0;
}



