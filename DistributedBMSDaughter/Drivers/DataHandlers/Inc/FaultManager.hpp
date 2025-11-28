/*
 * FaultManager.hpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #ifndef INC_FAULTMANAGER_HPP_
 #define INC_FAULTMANAGER_HPP_
 
 #pragma once
 #include <cstdint>
 #include <array>
 
 class FaultManager {
 public:
    enum class FaultType : uint8_t {
        BQ76920_COMM_ERROR = 0,
		BQ76920_RESULT_ERROR = 1,
		ADC_RESULT_ERROR = 2,
        CAN_TRANSMIT_ERROR = 3,


    };
 


 
     FaultManager();
     
     // Fault management
     void setFault(FaultType type);
     void clearFault(FaultType type);

     
     // Fault information
     uint8_t getFaultMask() const;

     

 
private:

     uint8_t fault_mask_;


 };
 
 #endif /* INC_FAULTMANAGER_HPP_ */
