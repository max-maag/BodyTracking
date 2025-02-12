#include "IKSolver.h"

#include <optional>
#include "Settings.h"

IKSolver::IKSolver(std::string_view name, unsigned int numIterationsMax, float thresholdTargetReachedPosition, float thresholdTargetReachedRotation) :
	name(name),
	numIterationsMax(numIterationsMax),
	thresholdTargetReachedPosition(thresholdTargetReachedPosition),
	thresholdTargetReachedRotation(thresholdTargetReachedRotation) {}

IKSolver::~IKSolver() {
	// Nothing to do.
}

void IKSolver::solve(BoneNode* boneEndEffector, Kore::vec3 positionTarget, Kore::Quaternion orientationTarget, IKEvaluator* evaluator) {
	auto stats = evaluator ? std::make_optional(evaluator->beginRun()) : std::nullopt;

	unsigned int numIterations = 0;

	
	float errorPosition = calculateErrorPosition(positionTarget, boneEndEffector->getPosition());
	float errorOrientation = calculateErrorRotation(orientationTarget, boneEndEffector->getOrientation());
	bool hasReachedTarget = checkHasReachedTarget(errorPosition, errorOrientation);
	bool isStuck = false;

	while (numIterations < numIterationsMax && !hasReachedTarget && !isStuck) {
		if (stats) {
			stats->beginIteration();
		}

		iterate(boneEndEffector, positionTarget, orientationTarget);

		float errorPositionLast = errorPosition;
		float errorOrientationLast = errorOrientation;

		errorPosition = calculateErrorPosition(positionTarget, boneEndEffector->getPosition());
		errorOrientation = calculateErrorRotation(orientationTarget, boneEndEffector->getOrientation());
		hasReachedTarget = checkHasReachedTarget(errorPosition, errorOrientation);

		isStuck = fabs(errorPositionLast - errorPosition) < Settings::nearNull || fabs(errorOrientationLast - errorOrientation) < Settings::nearNull;

		numIterations++;

		if (stats) {
			stats->endIteration();
		}
	}

	if (stats) {
		stats->endRun(errorPosition, errorOrientation, hasReachedTarget, isStuck);
	}
}

bool IKSolver::checkHasReachedTarget(float errorPosition, float errorRotation) const {
	return errorPosition < thresholdTargetReachedPosition && errorRotation < thresholdTargetReachedRotation;
}

float IKSolver::calculateErrorPosition(Kore::vec3 positionTarget, Kore::vec3 positionEndEffector) const {
	return positionTarget.distance(positionEndEffector);
}

float IKSolver::calculateErrorRotation(Kore::Quaternion rotationTarget, Kore::Quaternion rotationEndEffector) const {
	rotationTarget.normalize();
	Kore::Quaternion deltaRot_quat = rotationTarget.rotated(rotationEndEffector.invert());

	if (deltaRot_quat.w < 0) {
		deltaRot_quat = deltaRot_quat.scaled(-1);
	}

	Kore::vec3 deltaRot = Kore::vec3(0, 0, 0);

	Kore::RotationUtility::quatToEuler(&deltaRot_quat, &deltaRot.x(), &deltaRot.y(), &deltaRot.z());

	return Kore::RotationUtility::getDegree(deltaRot.getLength());
}

std::string_view IKSolver::getName() const {
	return name;
}

unsigned int IKSolver::getNumIterationsMax() const {
	return numIterationsMax;
}

float IKSolver::getThresholdTargetReachedPosition() const {
	return thresholdTargetReachedPosition;
}

float IKSolver::getThresholdTargetReachedRotation() const {
	return thresholdTargetReachedRotation;
}