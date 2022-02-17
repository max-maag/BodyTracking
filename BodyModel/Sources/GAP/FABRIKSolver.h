#ifndef _H_FABRIKSOLVER_
#define _H_FABRIKSOLVER_

#include <vector>
#include "../IKSolver.h"

class FABRIKSolver : public IKSolver {
protected:
	// IK chain root -> end effector
	std::vector<BoneNode*> mIKChain;

	std::vector<Kore::vec3> jointWorldPositions;

	// boneLength[0] == 0; boneLength[i] = jointWorldPositions[i] - joinWorldPositions[i-1]
	std::vector<float> boneLengths;
protected:
	void beforeIterations(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) override;
	void iterate(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) override;
	void afterIterations() override;

	void IterateForward(const Kore::vec3& goal);
	void IterateBackward(const Kore::vec3& base);

	// Sets IK chain transforms so that they match jointWOrldPositions
	void WorldToIKChain();
public:
	FABRIKSolver(unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation);
};


#endif // !_H_FABRIKSOLVER_
