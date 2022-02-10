#pragma once

#include <Kore/Math/Core.h>
#include <array>
#include <string>

#include "JacobianIKMode.h"

namespace Settings {
	extern const std::array<std::string, 7> files;
	extern const float nearNull;
	
	extern bool logRawData;

	extern const int numTrackers;
	extern const bool simpleIK; // Simple IK uses only 6 sensors (ignoring forearms)

	// Optimized IK Parameter
	extern const float optimalLambda[6];
	extern const float optimalErrorMaxPos[6];
	extern const float optimalErrorMaxRot[6];
	extern const float optimalMaxIterations[6];
    
    // Evaluation values
    extern const bool eval;
	extern const int evalMinIk;
	extern const int evalMaxIk;
}