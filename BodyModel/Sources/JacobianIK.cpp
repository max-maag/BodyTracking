#include "pch.h"
#include "JacobianIK.h"
#include "RotationUtility.h"

#include <Kore/System.h>
#include <array>
#include <vector>

using namespace Kore;


JacobianIK::JacobianIK(
	JacobianIKMode ikMode,
	unsigned int numIterationsMax,
	float thresholdTargetReachedPosition,
	float thresholdTargetReachedRotation
): IKSolver(JIK_MODE_NAMES[ikMode], numIterationsMax, thresholdTargetReachedPosition, thresholdTargetReachedRotation),
	ikMode(ikMode) {
}


void JacobianIK::iterate(BoneNode* targetBone, Kore::vec3 desPosition, Kore::Quaternion desRotation) {
	std::vector<float> deltaTheta;

	if (Settings::simpleIK && (targetBone->nodeIndex == leftHandBoneIndex || targetBone->nodeIndex == rightHandBoneIndex)) {
		deltaTheta = jacobianSimpleIKHand->calcDeltaTheta(targetBone, desPosition, desRotation, ikMode);
	} else if (!Settings::simpleIK && (targetBone->nodeIndex == leftForeArmBoneIndex || targetBone->nodeIndex == rightForeArmBoneIndex)) {
		deltaTheta = jacobianHand->calcDeltaTheta(targetBone, desPosition, desRotation, ikMode);
	} else if (targetBone->nodeIndex == leftFootBoneIndex|| targetBone->nodeIndex == rightFootBoneIndex) {
		deltaTheta = jacobianFoot->calcDeltaTheta(targetBone, desPosition, desRotation, ikMode);
	} else if (targetBone->nodeIndex == headBoneIndex) {
		deltaTheta = jacobianHead->calcDeltaTheta(targetBone, desPosition, desRotation, ikMode);
	}
		
	applyChanges(deltaTheta, targetBone);
	targetBone->applyJointConstraints();
	/*
	// update chain from root to end effector
	std::vector<BoneNode*> chain;
	chain.reserve(10);

	for (BoneNode* bone = targetBone; bone && bone->initialized; bone = bone->parent) {
		chain.push_back(bone);
	}

	for (auto rit = chain.rbegin(); rit != chain.rend(); ++rit) {
		(*rit)->update();
	}
	*/

	for (BoneNode* bone : bones) {
		bone->update();
	}
}


void JacobianIK::applyChanges(std::vector<float> deltaTheta, BoneNode* targetBone) {
	size_t size = deltaTheta.size();
	size_t i = 0;
	
	BoneNode* bone = targetBone;
	while (bone->initialized && i < size) {
		Kore::vec3 axes = bone->axes;
		
		if (axes.x() == 1.0 && i < size) bone->rotation.rotate(Kore::Quaternion(Kore::vec3(1, 0, 0), deltaTheta[i++]));
		if (axes.y() == 1.0 && i < size) bone->rotation.rotate(Kore::Quaternion(Kore::vec3(0, 1, 0), deltaTheta[i++]));
		if (axes.z() == 1.0 && i < size) bone->rotation.rotate(Kore::Quaternion(Kore::vec3(0, 0, 1), deltaTheta[i++]));
		
		bone->rotation.normalize();
		bone->local = bone->transform * bone->rotation.matrix().Transpose();
		
		bone = bone->parent;
	}
}

void JacobianIK::setBones(std::vector<BoneNode*> bones) {
	this->bones = bones;
}