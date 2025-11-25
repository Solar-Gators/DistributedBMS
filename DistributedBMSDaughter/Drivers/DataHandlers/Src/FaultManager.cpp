/*
 * FaultManager.cpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #include "FaultManager.hpp"
 #include <algorithm>
 
FaultManager::FaultManager() 
    : fault_mask_(0)
{
}
 
 void FaultManager::setFault(FaultType type) {
     
     uint8_t index = static_cast<uint8_t>(type);
	 fault_mask_ |= (1U << index);

 }
 
 void FaultManager::clearFault(FaultType type) {

	 uint8_t index = static_cast<uint8_t>(type);
	 fault_mask_ &= ~(1U << index);

 }
 

 
 uint8_t FaultManager::getFaultMask() const {
     return fault_mask_;
 }
 

