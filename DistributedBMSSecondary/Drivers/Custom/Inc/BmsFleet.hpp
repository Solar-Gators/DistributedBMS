/*
 * BmsFleet2.hpp
 *
 *  Created on: Nov 26, 2025
 *      Author: samrb
 */

#ifndef CUSTOM_INC_BMSFLEET_HPP_
#define CUSTOM_INC_BMSFLEET_HPP_

#define MAX_MODULES 8
#define STALE_MS 1500

#pragma once
#include <cstdint>
#include <array>
#include "CanFrames.hpp"
#include "CanBus.hpp"
//#include "CanDriver.hpp"

struct ModuleData {
	float highTemp = -1000;
	float avgTemp = 0;
	uint8_t highTempID = 0;

	uint16_t highVoltage = 0, lowVoltage = UINT16_MAX;
	uint8_t lowVoltageID = 0, highVoltageID = 0;

	float avgVoltage = 0;


	uint8_t num_cells = 0;
	uint8_t faults = 0;

	uint32_t last_ms = 0;

	bool online = false;

	void clear();


};

struct FleetData {
	uint32_t totalVoltage;
	uint16_t highestVoltage;
	uint16_t lowestVoltage;
	float highestTemp;

	//IDs should be in reference to the whole pack so we need to add up all the cells from previous modules
	uint8_t highVoltageID;
	uint8_t lowVoltageID;
	uint8_t highTempID;


};
struct IdMapElement {
	uint16_t can_id = 0;
	uint8_t index = 0;
	bool used = false;
};

class BmsFleet {
public:
	BmsFleet();

	bool registerDaughter(uint16_t can_id, uint8_t index);
	void handleMessage(const CanBus::Frame& msg, uint32_t now_ms);
	void processModules();

	ModuleData& module(uint8_t id);

	const FleetData& fleet() const { return fleet_; }

	void online(uint32_t now_ms);

	bool isOnline(uint8_t index);

	size_t packFleetData(uint8_t* out) const;

private:
	std::array<ModuleData, MAX_MODULES> modules_;
	std::array<IdMapElement, MAX_MODULES> idmap_;

	FleetData fleet_;



};


#endif /* CUSTOM_INC_BMSFLEET_HPP_ */
