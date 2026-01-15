/*
 * BmsFleet2.cpp
 *
 *  Created on: Nov 27, 2025
 *      Author: samrb
 */
#include <BmsFleet.hpp>
#include <cstring>
#include "cmsis_os.h"

#define stale 2000

void ModuleData::clear() {
	highTemp = -1000;
	avgTemp = 0;
	highTempID = 0;

	highVoltage = 0, lowVoltage = 0;
	lowVoltageID = 0, highVoltageID = 0;
	avgVoltage = 0;


}




BmsFleet::BmsFleet(){
	for(auto& m : modules_){
		m.clear();
	}
	for(auto& e : idmap_){
		e = {};
	}
}

void BmsFleet::online(uint32_t now_ms){
	for(auto& m : modules_){
		if(now_ms - m.last_ms > stale){
			m.online = false;
		}
	}

}

bool BmsFleet::isOnline(uint8_t index){
	auto& m = modules_[index];
	return(m.online);
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

	M.last_ms = osKernelGetTickCount();
	M.online = true;

	const uint8_t* data = msg.data;

	switch (CanFrames::getType(data)){
	case CanFrames::AVERAGES: {
		float avgTemp; uint16_t avgVoltage; uint8_t numCells;
		if (CanFrames::decodeAverages(data, avgTemp, avgVoltage, numCells)){

			M.avgTemp = avgTemp;
			M.avgVoltage = avgVoltage;
			M.num_cells = numCells;


		}

	}break;

	case CanFrames::VOLTAGE_EXTREMES: {
		uint16_t highVoltage, lowVoltage;
		uint8_t highIndex, lowIndex;
		uint8_t faults;
		if (CanFrames::decodeVoltageExtremes(data, highVoltage, lowVoltage, lowIndex, highIndex, faults)) {
			M.highVoltage = highVoltage;
			M.lowVoltage = lowVoltage;
			M.lowVoltageID = lowIndex;
			M.highVoltageID = highIndex;
			M.faults = faults;

		}
	}break;

	case CanFrames::HIGH_TEMP: {
		float highTemp; uint8_t highIndex;
		if(CanFrames::decodeHighTemp(data, highTemp, highIndex)){
			M.highTemp = highTemp;
			M.highTempID = highIndex;

		}
	}break;

	default:
		break;
	}


}

void BmsFleet::processModules() {
    FleetData fleet{};
    fleet.totalVoltage   = 0;
    fleet.highestVoltage = 0;
    fleet.lowestVoltage  = 0xFFFF;      // initialize high so first real value replaces it
    fleet.highestTemp    = -1000;

    uint8_t cellOffset = 0;             // used to compute global cell indexes

    uint32_t now_ms = osKernelGetTickCount();    // Or whatever timestamp you prefer
    online(now_ms);

    for (size_t i = 0; i < modules_.size(); i++) {
        auto& m = modules_[i];

        // Skip stale/offline modules
        if (!m.online)
        {
            cellOffset += m.num_cells;  // still advance offset so indexing stays consistent
            continue;
        }

        // 1. Total pack voltage
        fleet.totalVoltage += m.avgVoltage * m.num_cells;

        // 2. Highest cell voltage in pack
        if (m.highVoltage > fleet.highestVoltage) {
            fleet.highestVoltage = m.highVoltage;
            fleet.highVoltageID  = cellOffset + m.highVoltageID;
        }

        // 3. Lowest cell voltage in pack
        if (m.lowVoltage < fleet.lowestVoltage) {
            fleet.lowestVoltage = m.lowVoltage;
            fleet.lowVoltageID  = cellOffset + m.lowVoltageID;
        }

        // 4. Highest temperature in pack
        if (m.highTemp > fleet.highestTemp) {
            fleet.highestTemp = m.highTemp;
            fleet.highTempID  = cellOffset + m.highTempID;
        }

        // increase global cell offset for next module
        cellOffset += m.num_cells;
    }

    fleet_ = fleet;
}


/*
size_t BmsFleet::packFleetData(uint8_t* out) const
{
    size_t offset = 0;
    const FleetData& f = fleet_;

    memcpy(out + offset, &f.totalVoltage, sizeof(f.totalVoltage));
    offset += sizeof(f.totalVoltage);

    memcpy(out + offset, &f.highestVoltage, sizeof(f.highestVoltage));
    offset += sizeof(f.highestVoltage);

    memcpy(out + offset, &f.lowestvoltage, sizeof(f.lowestvoltage));
    offset += sizeof(f.lowestvoltage);

    memcpy(out + offset, &f.highestTemp, sizeof(f.highestTemp));
    offset += sizeof(f.highestTemp);

    memcpy(out + offset, &f.highVoltageID, sizeof(f.highVoltageID));
    offset += sizeof(f.highVoltageID);

    memcpy(out + offset, &f.lowVoltageID, sizeof(f.lowVoltageID));
    offset += sizeof(f.lowVoltageID);

    memcpy(out + offset, &f.highTempID, sizeof(f.highTempID));
    offset += sizeof(f.highTempID);

    out[offset++] = '\r';
    out[offset++] = '\n';

    return offset; // = 13 bytes
}
*/
size_t BmsFleet::packFleetData(uint8_t* out) const
{
    size_t offset = 0;
    const FleetData& f = fleet_;

    // ---- UART framing header ----
    out[offset++] = 0xA5;
    out[offset++] = 0x5A;

    // Reserve length field
    size_t length_offset = offset;
    out[offset++] = 0x00; // length LSB
    out[offset++] = 0x00; // length MSB

    size_t payload_start = offset;

    // ---- Payload ----
    out[offset++] = 17;  // payload[0] = message ID

    auto put_u16 = [&](uint16_t v) {
        out[offset++] = v & 0xFF;
        out[offset++] = (v >> 8) & 0xFF;
    };

    auto put_u8 = [&](uint8_t v) {
        out[offset++] = v;
    };

    put_u16(f.totalVoltage);
    put_u16(f.highestVoltage);
    put_u16(f.lowestVoltage);
    put_u16(f.highestTemp);

    put_u8(f.highVoltageID);
    put_u8(f.lowVoltageID);
    put_u8(f.highTempID);

    // ---- Finalize payload length ----
    uint16_t payload_len = offset - payload_start;
    out[length_offset + 0] = payload_len & 0xFF;
    out[length_offset + 1] = (payload_len >> 8) & 0xFF;

    return offset; // total UART frame length
}



