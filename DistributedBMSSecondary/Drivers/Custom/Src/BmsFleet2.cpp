/*
 * BmsFleet2.cpp
 *
 *  Created on: Nov 27, 2025
 *      Author: samrb
 */
#include "BmsFleet2.hpp"

void ModuleData::clear() {
	highTemp = -1000;
	avgTemp = 0;
	highTempID = 0;

	highVoltage = 0, lowVoltage = 0;
	lowVoltageID = 0, highVoltageID = 0;
	avgVoltage = 0;


}

bool ModuleData::online(uint32_t now_ms, uint32_t stale_ms){
    return (now_ms - last_ms) <= stale_ms;
}


BmsFleet::BmsFleet(){
	for(auto& m : modules_){
		m.clear();
	}
	for(auto& e : indexMap_){
		e = {};
	}
}

bool BmsFleet::register_node(uint16_t can_id, uint8_t index){
	for(auto& e: idmap_){
		if(!e.used){
			e.used = true;
			e.can_id = can_id;
			e.index = index;
			return true;
		}
	}
	return false;
}

void BmsFleet::handleMessage(const CANDriver::CANFrame& msg, uint32_t now_ms){
	int moduleIndex = (uint16_t)msg.can_id = 0x100;

}
