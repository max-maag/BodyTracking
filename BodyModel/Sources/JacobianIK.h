#pragma once

#include "Jacobian.h"

#include <memory>
#include <Kore/Math/Quaternion.h>

#include "JacobianIKMode.h"
#include "IKSolver.h"
#include "IKEvaluator.h"

class JacobianIK: public IKSolver {
	
public:
	JacobianIK(
		JacobianIKMode ikMode,
		unsigned int numIterationsMax,
		float thresholdTargetReachedPosition,
		float thresholdTargetReachedRotation);

	void setBones(std::vector<BoneNode*> bones);

protected:
	void iterate(BoneNode* targetBone, Kore::vec3 desPosition, Kore::Quaternion desRotation) override;

private:
	std::vector<BoneNode*> bones;
	JacobianIKMode ikMode;
	
	static const int handJointSimpleIKDOFs = 7;
	Jacobian<handJointSimpleIKDOFs>* jacobianSimpleIKHand = new Jacobian<handJointSimpleIKDOFs>;
	static const int handJointDOFs = 4;
	Jacobian<handJointDOFs>* jacobianHand = new Jacobian<handJointDOFs>;
	
	static const int footJointDOFs = 4;
	Jacobian<footJointDOFs>* jacobianFoot = new Jacobian<footJointDOFs>;
	
	static const int headJointDOFs = 5;
	Jacobian<headJointDOFs>* jacobianHead = new Jacobian<headJointDOFs>;
	
	
	void applyChanges(std::vector<float> deltaTheta, BoneNode* targetBone);
};
