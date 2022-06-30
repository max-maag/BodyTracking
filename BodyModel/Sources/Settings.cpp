#include "Settings.h"
#include "Kore/Math/Core.h"

const std::array<std::string, 7> Settings::files{ "walking.csv", "lunges.csv", "squats.csv", "kicks.csv", "yoga1.csv", "yoga2.csv", "yoga3.csv" };
//const int numFiles = 16;
//const char* files[numFiles] = { "sitting.csv", "standing.csv", "walking_long.csv", "shoulder_forward_flexion.csv", "shoulder_abduction.csv", "shoulder_horizontal_abduction.csv",  "rotation_with_arm_at_side.csv", "rotation_with_arm_in_abduction.csv", "elbow_flexion.csv", "hand_pronation.csv", "knee_flexion.csv", "hip_flexion.csv", "punching.csv", "kicking.csv", "squats.csv", "lunges.csv" };

const float Settings::nearNull = 0.000001f;

bool Settings::logRawData = false;

const int Settings::numTrackers = 3;
const bool Settings::simpleIK = true; // Simple IK uses only 6 sensors (ignoring forearms)

// Optimized IK Parameter
//													JT = 0		JPI = 1		DLS = 2		SVD = 3		SVD_DLS = 4		SDLS = 5
const float Settings::optimalLambda[6] = { 0.35f,		0.05f,		0.25f,		0.03f,		0.25f,			1.0f / 12.0f * Kore::pi };
const float Settings::optimalErrorMaxPos[6] = { 0.01f,		0.1f,		0.001f,		0.01f,		0.001f,			0.01f };
const float Settings::optimalErrorMaxRot[6] = { 0.01f,		0.1f,		0.01f,		0.01f,		0.01f,			0.01f };
const float Settings::optimalMaxIterations[6] = { 30.0f,		4000.0f,	20.0f,		10.0f,		20.0f,			20.0f };

// Evaluation values
const bool Settings::eval = false;
const int Settings::evalMinIk = 0;
const int Settings::evalMaxIk = 5;