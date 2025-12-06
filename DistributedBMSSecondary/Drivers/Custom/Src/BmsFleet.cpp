/*
 * BmsFleet2.cpp
 *
 *  Created on: Nov 27, 2025
 *      Author: samrb
 */
#include <BmsFleet.hpp>

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
	for(auto& e : idmap_){
		e = {};
	}
}

bool BmsFleet::registerDaughter(uint16_t can_id, uint8_t index){
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

ModuleData& BmsFleet::module(uint8_t id){
	return modules_[id];
}

void BmsFleet::handleMessage(const CanBus::Frame& msg, uint32_t now_ms){
	int moduleIndex = (uint16_t)msg.id - 0x100;
	if(moduleIndex < 0) return;

	ModuleData& M = modules_[moduleIndex];


	const uint8_t* data = msg.data;

	switch (CanFrames::getType(data)){
	case CanFrames::AVERAGES: {
		float avgTemp; uint16_t avgVoltage; uint8_t numCells; uint8_t faults;
		if (CanFrames::decodeAverages(data, avgTemp, avgVoltage, numCells)){

			M.avgTemp = avgTemp;
			M.avgVoltage = avgVoltage;
			M.num_cells = numCells;
			M.last_ms = now_ms;

		}

	}break;

	case CanFrames::VOLTAGE_EXTREMES: {
		uint16_t highVoltage, lowVoltage;
		uint8_t highIndex, lowIndex;
		if (CanFrames::decodeVoltageExtremes(data, highVoltage, lowVoltage, lowIndex, highIndex)) {
			M.highVoltage = highVoltage;
			M.lowVoltage = lowVoltage;
			M.lowVoltageID = lowIndex;
			M.highVoltageID = highIndex;
			M.last_ms = now_ms;
		}
	}

	default:
		break;
	}


}

