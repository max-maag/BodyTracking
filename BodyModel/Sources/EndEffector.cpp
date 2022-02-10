#include "pch.h"
#include "EndEffector.h"

#include <assert.h>

#include <Kore/Log.h>

#include <string>

EndEffector::EndEffector(int boneIndex) : desPosition(Kore::vec3(0, 0, 0)), desRotation(Kore::Quaternion(0, 0, 0, 1)), offsetPosition(Kore::vec3(0, 0, 0)), offsetRotation(Kore::Quaternion(0, 0, 0, 1)), finalPosition(Kore::vec3(0, 0, 0)), finalRotation(Kore::Quaternion(0, 0, 0, 1)),  boneIndex(boneIndex), deviceID(-1) {
	name = getNameForIndex(boneIndex);
}

Kore::vec3 EndEffector::getDesPosition() const {
	return desPosition;
}

void EndEffector::setDesPosition(Kore::vec3 pos) {
	desPosition = pos;
}

Kore::Quaternion EndEffector::getDesRotation() const {
	return desRotation;
}

void EndEffector::setDesRotation(Kore::Quaternion rot) {
	desRotation = rot;
}

Kore::vec3 EndEffector::getOffsetPosition() const {
	return offsetPosition;
}

void EndEffector::setOffsetPosition(Kore::vec3 pos) {
	offsetPosition = pos;
}

Kore::Quaternion EndEffector::getOffsetRotation() const {
	return offsetRotation;
}

void EndEffector::setOffsetRotation(Kore::Quaternion rot) {
	offsetRotation = rot;
}

Kore::vec3 EndEffector::getFinalPosition() const {
	return finalPosition;
}

void EndEffector::setFinalPosition(Kore::vec3 pos) {
	finalPosition = pos;
}

Kore::Quaternion EndEffector::getFinalRotation() const {
	return finalRotation;
}

void EndEffector::setFinalRotation(Kore::Quaternion rot) {
	finalRotation = rot;
}

int EndEffector::getDeviceIndex() const {
	return deviceID;
}

void EndEffector::setDeviceIndex(int index) {
	deviceID = index;
}

int EndEffector::getBoneIndex() const {
	return boneIndex;
}

const char* EndEffector::getName() const {
	return name;
}

const char* EndEffector::getNameForIndex(const int ID) const {
	if (ID == headBoneIndex)				return headTag;
	else if (ID == hipBoneIndex)			return hipTag;
	else if (ID == leftHandBoneIndex)		return lHandTag;
	else if (ID == rightHandBoneIndex)		return rHandTag;
	else if (ID == leftForeArmBoneIndex)	return lForeArm;
	else if (ID == rightForeArmBoneIndex)	return rForeArm;
	else if (ID == leftFootBoneIndex)		return lFootTag;
	else if (ID == rightFootBoneIndex)		return rFootTag;
	else if (ID == leftLegBoneIndex)		return lKneeTag;
	else if (ID == rightLegBoneIndex)		return rKneeTag;
	else return nullptr;
}

int EndEffector::getIndexForName(const char* name) const {
	if(std::strcmp(name, headTag) == 0)			return headBoneIndex;
	else if(std::strcmp(name, hipTag) == 0)		return hipBoneIndex;
	else if(std::strcmp(name, lHandTag) == 0)	return leftHandBoneIndex;
	else if(std::strcmp(name, rHandTag) == 0)	return rightHandBoneIndex;
	else if (std::strcmp(name, lForeArm) == 0)	return leftForeArmBoneIndex;
	else if (std::strcmp(name, rForeArm) == 0)	return rightForeArmBoneIndex;
	else if (std::strcmp(name, lFootTag) == 0)	return leftFootBoneIndex;
	else if (std::strcmp(name, rFootTag) == 0)	return rightFootBoneIndex;
	else if (std::strcmp(name, lKneeTag) == 0)	return leftLegBoneIndex;
	else if (std::strcmp(name, rKneeTag) == 0)	return rightLegBoneIndex;
	else return -1;
}
