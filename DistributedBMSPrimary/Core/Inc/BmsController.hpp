/*
 * BmsController.hpp
 *
 *  Created on: Nov 19, 2025
 *      Author: samrb
 */

#ifndef INC_BMSCONTROLLER_HPP_
#define INC_BMSCONTROLLER_HPP_


#pragma once
#include "PrimaryBMSFleet.hpp"

class BmsController {
public:

	bool contactors_on;
	uint8_t faults;
	void updateFaultsFromSummary(FleetSummaryData summary);




};


#endif /* INC_BMSCONTROLLER_HPP_ */
