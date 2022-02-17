#pragma once

#include <memory>
#include <string_view>
#include <Kore/Math/Vector.h>
#include <Kore/Math/Quaternion.h>

#include "MeshObject.h"
#include "IKEvaluator.h"

class IKSolver
{
private:
	const std::string_view name;
	const unsigned int numIterationsMax;

	// If end effector is closer to target that this distance, it has reached its target's position.
	const float thresholdTargetReachedPosition;

	// If rotation error metric is smaller than this threshold, the end effector has reached its target's rotation.
	const float thresholdTargetReachedRotation;

	float calculateErrorPosition(Kore::vec3 positionTarget, Kore::vec3 positionEndEffector) const;
	float calculateErrorRotation(Kore::Quaternion rotationTarget, Kore::Quaternion rotationEndEffector) const;
	bool checkHasReachedTarget(float errorPosition, float errorRotation) const;

protected:
	// Executed once before the iteration process starts. Perform iteration setup here.
	virtual void beforeIterations(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget);

	virtual void iterate(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget) = 0;

	// Executed once after the iterations are done. Perform clean-up here.
	virtual void afterIterations();

public:
	IKSolver(std::string_view name, unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation);
	virtual ~IKSolver();
	
	void solve(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget, IKEvaluator* evaluator);

	std::string_view getName() const;
	
	unsigned int getNumIterationsMax() const;

	float getThresholdTargetReachedPosition() const;

	float getThresholdTargetReachedRotation() const;

};

