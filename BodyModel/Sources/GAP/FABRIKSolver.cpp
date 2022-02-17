#include "FABRIKSolver.h"

#include <algorithm>
#include <Kore/Log.h>

#include "QuatUtils.h"

FABRIKSolver::FABRIKSolver(unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation):
	IKSolver("FABRIK", numIterationsMax, thresholdTargetReachedPosition, thresholdTargetReachedRotation) {
}


void FABRIKSolver::WorldToIKChain() {
	size_t size = mIKChain.size();

	for (size_t i = 0; i < size - 1; ++i) {
		Kore::vec3 position = mIKChain[i]->getPosition();
		Kore::Quaternion rotation = mIKChain[i]->getOrientation();
		Kore::mat3 rotMat(rotation.invert().matrix());

		Kore::vec3 toNext = mIKChain[i+1]->getPosition()  - position;
		toNext = rotMat * toNext;

		Kore::vec3 toDesired = jointWorldPositions[i + 1] - position;
		toDesired = rotMat * toDesired;

		Kore::Quaternion delta = Kore::RotationUtility::fromTo(toNext, toDesired);
		mIKChain[i]->rotation = delta * mIKChain[i]->rotation;

		mIKChain[i]->calculateLocal();
		mIKChain[i]->update();
	}
}

void FABRIKSolver::beforeIterations(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) {
	for (BoneNode* bone = boneEndEffector; bone && bone->initialized; bone = bone->parent) {
		mIKChain.push_back(bone);
	}

	std::reverse(mIKChain.begin(), mIKChain.end());

	boneLengths.resize(mIKChain.size());
	jointWorldPositions.resize(mIKChain.size());

	Kore::vec3 positionWorldLast = mIKChain.front()->getPosition();

	for (size_t idxBone = 0; idxBone < mIKChain.size(); ++idxBone) {
		jointWorldPositions[idxBone] = mIKChain[idxBone]->getPosition();
		
		boneLengths[idxBone] = jointWorldPositions[idxBone].distance(positionWorldLast);

		positionWorldLast = jointWorldPositions[idxBone];
	}

	if (mIKChain.empty()) {
		Kore::log(Kore::LogLevel::Info, "Solve called on empty chain!");
	}
}

void FABRIKSolver::iterate(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) {
	if (mIKChain.empty()) {
		return;
	}

	IterateBackward(positionTarget);
	IterateForward(mIKChain.front()->getPosition());

	WorldToIKChain();
}

void FABRIKSolver::afterIterations() {
	mIKChain.clear();
	boneLengths.clear();
}

void FABRIKSolver::IterateBackward(const Kore::vec3& goal) {
	size_t size = mIKChain.size();

	if (size > 0) {
		jointWorldPositions[size - 1] = goal;

		for (size_t i = size - 1; i > 0; --i) {
			Kore::vec3 direction = (jointWorldPositions[i - 1] - jointWorldPositions[i]).normalize();
			Kore::vec3 offset = direction * boneLengths[i];
			jointWorldPositions[i - 1] = jointWorldPositions[i] + offset;
		}
	}
}

void FABRIKSolver::IterateForward(const Kore::vec3& base) {
	size_t size = mIKChain.size();

	if (size > 0) {
		jointWorldPositions[0] = base;
	}

	for (size_t i = 1; i < size; ++i) {
		Kore::vec3 direction = (jointWorldPositions[i] - jointWorldPositions[i - 1]).normalize();
		Kore::vec3 offset = direction * boneLengths[i];
		jointWorldPositions[i] = jointWorldPositions[i - 1] + offset;
	}
}

