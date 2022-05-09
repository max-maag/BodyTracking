#include "FABRIKSolver.h"

#include <algorithm>
#include <Kore/Log.h>

#include <cmath>
#include "QuatUtils.h"

FABRIKSolver::FABRIKSolver(unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation):
	IKSolver("FABRIK", numIterationsMax, thresholdTargetReachedPosition, thresholdTargetReachedRotation) {
}

//*
void FABRIKSolver::WorldToIKChain() {
	Kore::log(Kore::LogLevel::Info, "");

	size_t size = mIKChain.size();

	for (size_t i = 0; i < size - 1; ++i) {
		Kore::vec3 positionActual = mIKChain[i]->getPosition();
		Kore::Quaternion rotationActual = mIKChain[i]->getOrientation();
		
		Kore::vec3 positionParent = mIKChain[i]->parent->getPosition();

		Kore::vec3 directionActual = positionActual - positionParent;

		Kore::vec3 directionDesired = jointWorldPositions[i] - positionParent;
		

		//Kore::Quaternion delta = Kore::RotationUtility::getRotationFromTo(directionActual, directionDesired);
		
		Kore::Quaternion delta = Kore::RotationUtility::fromTo(directionActual, directionDesired);

		Kore::log(Kore::LogLevel::Info, "%-40s%s", "FABRIKSolver::WorldToIKChain", mIKChain[i]->boneName);
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "parent position", positionParent.x(), positionParent.y(), positionParent.z());
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "actual position", positionActual.x(), positionActual.y(), positionActual.z());
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "desired position", jointWorldPositions[i].x(), jointWorldPositions[i].y(), jointWorldPositions[i].z());
		
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "actual direction", directionActual.x(), directionActual.y(), directionActual.z());
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "desired direction", directionDesired.x(), directionDesired.y(), directionDesired.z());

		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "local rotation", mIKChain[i]->rotation.x, mIKChain[i]->rotation.y, mIKChain[i]->rotation.z, mIKChain[i]->rotation.w);
		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "delta", delta.x, delta.y, delta.z, delta.w);

		mIKChain[i]->rotation.rotate(delta);

		Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f  % .6f", "FABRIKSolver::WorldToIKChain", "local rotation after", mIKChain[i]->rotation.x, mIKChain[i]->rotation.y, mIKChain[i]->rotation.z, mIKChain[i]->rotation.w);

		mIKChain[i]->calculateLocal();
		mIKChain[i]->update();
	}
}
//*/
/*
void FABRIKSolver::WorldToIKChain() {
	size_t size = mIKChain.size();

	for (size_t i = 0; i < size - 1; ++i) {
		Kore::vec3 positionActual = mIKChain[i]->parent->getPosition();
		Kore::Quaternion rotationActual = mIKChain[i]->parent->getOrientation();
		Kore::mat3 rotMat(rotationActual.invert().matrix());

		Kore::vec3 toNext = mIKChain[i]->getPosition() - positionActual;
		toNext = rotMat * toNext;

		Kore::vec3 toDesired = jointWorldPositions[i] - positionActual;
		toDesired = rotMat * toDesired;

		Kore::Quaternion delta = Kore::RotationUtility::fromTo(toNext, toDesired);
		mIKChain[i]->rotation.rotate(delta);

		mIKChain[i]->calculateLocal();
		mIKChain[i]->update();
	}
}
//*/

void FABRIKSolver::beforeIterations(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) {
	Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::beforeIterations", boneEndEffector->boneName, positionTarget.x(), positionTarget.y(), positionTarget.z());
	for (BoneNode* bone = boneEndEffector; bone && bone->initialized; bone = bone->parent) {
		mIKChain.push_back(bone);
	}

	// Remove root bone, because it's a dummy bone.
	mIKChain.pop_back();

	// Remove hip bone, since it is fixed
	mIKChain.pop_back();

	std::reverse(mIKChain.begin(), mIKChain.end());

	boneLengths.resize(mIKChain.size());
	jointWorldPositions.resize(mIKChain.size());

	rootWorldPosition = mIKChain.front()->getPosition();
	Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::beforeIterations", "root position", rootWorldPosition.x(), rootWorldPosition.y(), rootWorldPosition.z());

	// Position of the previous bone, used to calculate bone length.
	Kore::vec3 positionWorldPrevious = rootWorldPosition;

	for (size_t idxBone = 0; idxBone < mIKChain.size(); ++idxBone) {
		jointWorldPositions[idxBone] = mIKChain[idxBone]->getPosition();
		
		boneLengths[idxBone] = jointWorldPositions[idxBone].distance(positionWorldPrevious);

		positionWorldPrevious = jointWorldPositions[idxBone];
	}

	if (mIKChain.empty()) {
		Kore::log(Kore::LogLevel::Info, "Solve called on empty chain!");
	}
}

void FABRIKSolver::iterate(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) {
	Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::iterate", boneEndEffector->boneName, positionTarget.x(), positionTarget.y(), positionTarget.z());
	if (mIKChain.empty()) {
		return;
	}

	IterateBackward(positionTarget);
	IterateForward(rootWorldPosition);

	fireEvent(EventType::IterationComplete);
}

void FABRIKSolver::afterIterations() {
	Kore::log(Kore::LogLevel::Info, "%-40s", "FABRIKSolver::afterIterations");
	if (mIKChain.empty()) {
		return;
	}

	fireEvent(EventType::SolveComplete);

	WorldToIKChain();

	mIKChain.clear();
	boneLengths.clear();
}

void FABRIKSolver::IterateBackward(const Kore::vec3& goal) {
	Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::IterateBackward", "", goal.x(), goal.y(), goal.z());

	size_t size = mIKChain.size();

	if (size > 0) {
		jointWorldPositions[size - 1] = goal;

		fireEvent(EventType::IterationStepComplete);

		for (size_t i = size - 1; i > 0; --i) {
			Kore::vec3 direction = (jointWorldPositions[i - 1] - jointWorldPositions[i]).normalize();
			Kore::vec3 offset = direction * boneLengths[i];
			jointWorldPositions[i - 1] = jointWorldPositions[i] + offset;

			fireEvent(EventType::IterationStepComplete);
		}
	}
}

void FABRIKSolver::IterateForward(const Kore::vec3& base) {
	Kore::log(Kore::LogLevel::Info, "%-40s%-40s% .6f  % .6f  % .6f", "FABRIKSolver::IterateForward", "", base.x(), base.y(), base.z());
	size_t size = mIKChain.size();

	if (size > 0) {
		jointWorldPositions[0] = base;

		fireEvent(EventType::IterationStepComplete);
	}

	for (size_t i = 1; i < size; ++i) {
		Kore::vec3 direction = (jointWorldPositions[i] - jointWorldPositions[i - 1]).normalize();
		Kore::vec3 offset = direction * boneLengths[i];
		jointWorldPositions[i] = jointWorldPositions[i - 1] + offset;

		fireEvent(EventType::IterationStepComplete);
	}
}

void FABRIKSolver::addListener(EventListener listener) {
	eventListeners.push_back(listener);
}
/* This is not guaranteed to work
void FABRIKSolver::removeListener(EventListener& listener) {
	std::erase_if(eventListeners, [&listener](const EventListener& l) { return &listener == &l; });
}
*/

void FABRIKSolver::fireEvent(EventType eventType) {
	if (eventListeners.empty()) {
		return;
	}

	std::unordered_map<const BoneNode*, Kore::vec3> bonePositions;

	for (size_t idxBone = 0; idxBone < mIKChain.size(); ++idxBone) {
		bonePositions.emplace(mIKChain[idxBone], jointWorldPositions[idxBone]);
	}

	for (EventListener listener : eventListeners) {
		listener(eventType, bonePositions);
	}
}