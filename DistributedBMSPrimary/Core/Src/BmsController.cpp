/*
 * BmsController.cpp
 *
 *  Created on: Nov 19, 2025
 *      Author: samrb
 */
#include "BmsController.hpp"
#include "PrimaryBmsFleet.hpp"

void BmsController::updateFaultsFromSummary(FleetSummaryData summary){
	if(summary.highest_cell_mV > 4200){
		faults = faults | 0b00000001;
	}else{
		faults = faults & 0b11111110;
	}
	if(summary.lowest_cell_mV < 2500){
		faults = faults | 0b00000010;
	}else{
		faults = faults & 0b11111101;
	}
	if(summary.highest_temp_C > 45){
		faults = faults | 0b00000100;
	}else{
		faults = faults & 0b11111011;
	}
}


