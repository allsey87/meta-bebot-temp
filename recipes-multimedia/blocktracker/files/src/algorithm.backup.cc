       
   int8_t punLiftSpeeds[] = {64, -64};
   uint8_t punData[2];
		
   std::vector<SBlock> vecBlocks;

   punData[0] = reinterpret_cast<uint8_t&>(punLiftSpeeds[0]);
   punData[1] = reinterpret_cast<uint8_t&>(punLiftSpeeds[1]);

   /* for storing remote sensor data */
   bool bLiftAtBottom = false, 
      bLiftAtTop = false;
   uint16_t unRfRangeLeft = 0, 
      unRfRangeRight = 0, 
      unRfRangeFront = 0,
      unRfRangeUnderneath = 0;

   /* switch on actuator power for DDS */
   m_cPowerManagementUARTSocket.Write(reinterpret_cast<const uint8_t*>("A"), 1);

   /* Set DDS parameters (PID) */
   uint8_t params[] = {3,1,4};
   int8_t pnSpeed[2] = {5,-5};
   m_cSensorActuatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_PARAMS, params, 3);
   m_cSensorActuatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED, reinterpret_cast<const uint8_t*>(pnSpeed), 2);
   /* Switch on DDS */
   m_cSensorActuatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, punSwitchOn, 1);
      
   //m_cManipulatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED, &punData[0], 1);

   uint32_t steps = 0;
   double last_t = tic();
   for(;;) {

      /* sample sensors */
      m_cISSCaptureDevice >> image_gray;
      m_cManipulatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE);
      m_cManipulatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_RF_RANGE);
			
      vecBlocks.clear();
      processImage(image_gray, vecBlocks);

      bool bWaitingForRfResponse = true;
      bool bWaitingForSwitchResponse = true;

      do {
         m_cManipulatorInterface.ProcessInput();
         if(m_cManipulatorInterface.GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
            const CPacketControlInterface::CPacket& cPacket = m_cManipulatorInterface.GetPacket();
            switch(cPacket.GetType()) {
            case CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE:
               if(cPacket.GetDataLength() == 2) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  bLiftAtTop = (punPacketData[0] != 0);
                  bLiftAtBottom = (punPacketData[1] != 0);		
                  bWaitingForSwitchResponse = false;
               }
               break;
            case CPacketControlInterface::CPacket::EType::GET_RF_RANGE:
               if(cPacket.GetDataLength() == 8) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  unRfRangeLeft = (punPacketData[0] << 8) | punPacketData[1];
                  unRfRangeRight = (punPacketData[2] << 8) | punPacketData[3];
                  unRfRangeFront = (punPacketData[4] << 8) | punPacketData[5];
                  unRfRangeUnderneath = (punPacketData[6] << 8) | punPacketData[7];
                  bWaitingForRfResponse = false;
               }
               break;
            default:
               continue;
            }
         }
      } while((bWaitingForRfResponse || bWaitingForSwitchResponse) && !bInterruptEvent);

      std::cout << "[Range Finders]" << std::endl;
      std::cout << "R(L) = " << unRfRangeLeft << ", "
                << "R(R) = " << unRfRangeRight << ", "
                << "R(F) = " << unRfRangeFront << ", "
                << "R(U) = " << unRfRangeUnderneath << std::endl;
      std::cout << "[Limit Switches]" << std::endl;
      std::cout << "S(T) = " << (bLiftAtTop ? '1' : '0') << ", "
                << "S(B) = " << (bLiftAtBottom ? '1' : '0') << std::endl;
      std::cout << "[Blocks]" << std::endl;
      for(SBlock& sBlock : vecBlocks) {
         std::cout << "[" 
                   << sBlock.X << ", " 
                   << sBlock.Y << ", " 
                   << sBlock.Z << "] [" 
                   << sBlock.Yaw << ", " 
                   << sBlock.Pitch << ", " 
                   << sBlock.Roll << "]" 
                   << std::endl;
      }
			
      /* send the image to the host */
      m_cTCPImageSocket << image_gray;

      steps++;
      if (steps % 10 == 0) {
         double t = tic();
         cout << "-- " << 10./(t-last_t) << " control steps per second --" << endl;
         last_t = t;
      }

      if(bInterruptEvent) {
         cout << "-- shutdown request acknowledged --" << endl;
         break;
      }
   }

   /* Shutdown the DDS */
   m_cSensorActuatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, punSwitchOff, 1);
   /* switch off actuator power for DDS */
   m_cPowerManagementUARTSocket.Write(reinterpret_cast<const uint8_t*>("a"), 1);
   //m_cManipulatorInterface.SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED, &punData[1], 1);
