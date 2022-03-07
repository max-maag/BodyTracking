#include "FABRIKSolver.h"

#include <algorithm>
#include <Kore/Log.h>

#include "QuatUtils.h"

/* TODO:
 *  - Does the origin -> root bone need to be stored in mIKChain?
 *		- It doesn't need updating since root position and orientation shouldn't change.
 *		- worldPos[0] can be calculated using bone->parent
 *  - Check WorldToIKChain
 */

FABRIKSolver::FABRIKSolver(unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation):
	IKSolver("FABRIK", numIterationsMax, thresholdTargetReachedPosition, thresholdTargetReachedRotation) {
}


void FABRIKSolver::WorldToIKChain() {
	size_t size = mIKChain.size();

	for (size_t i = 0; i < size - 1; ++i) {
		Kore::vec3 positionActual = mIKChain[i]->getPosition();
		Kore::Quaternion rotationActual = mIKChain[i]->getOrientation();
		
		//Kore::mat3 rotActualInverted(rotationActual.invert().matrix());

		Kore::vec3 toNext = mIKChain[i+1]->getPosition()  - positionActual;
		//toNext = rotActualInverted * toNext;

		Kore::vec3 toDesired = jointWorldPositions[i + 1] - positionActual;
		//toDesired = rotActualInverted * toDesired;

		Kore::Quaternion delta = Kore::RotationUtility::getRotationFromTo(toNext, toDesired);//Kore::RotationUtility::fromTo(toNext, toDesired);
		//mIKChain[i]->rotation = delta * mIKChain[i]->rotation;
		mIKChain[i]->rotation = delta.rotated(mIKChain[i]->rotation);

		mIKChain[i]->calculateLocal();
		mIKChain[i]->update();
	}
}

void FABRIKSolver::beforeIterations(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) {
	for (BoneNode* bone = boneEndEffector; bone && bone->initialized; bone = bone->parent) {
		mIKChain.push_back(bone);
	}

	// Remove root bone, because it's a dummy bone.
	mIKChain.pop_back();

	std::reverse(mIKChain.begin(), mIKChain.end());

	boneLengths.resize(mIKChain.size());
	jointWorldPositions.resize(mIKChain.size());

	rootWorldPosition = mIKChain.front()->getPosition();

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
	if (mIKChain.empty()) {
		return;
	}

	IterateBackward(positionTarget);
	IterateForward(rootWorldPosition);
}

void FABRIKSolver::afterIterations() {
	WorldToIKChain();

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