#ifndef _H_FABRIKSOLVER_
#define _H_FABRIKSOLVER_

#include <vector>
#include "../IKSolver.h"
#include <unordered_map>
#include <functional>

class FABRIKSolver : public IKSolver {
public:

	struct DebugEvent {
		enum class Type {
			SolveBegin, IterationStepComplete, IterationComplete, SolveComplete
		};

		Type eventType;

		std::vector<BoneNode*> chain;
		BoneNode* bone = nullptr;
		Kore::vec3 position;
		Kore::vec3 positionBefore;
		Kore::vec3 positionTarget;
		std::vector<Kore::vec3> jointPositions;
	};

	using EventListener = std::function<void(DebugEvent)>;

private:
	/* The IK chain.
	 * 
	 * The first bone points from the origin to the first joint.
	 * The last bone points from the last joint to the target (I think). 
	 */
	std::vector<BoneNode*> mIKChain;

	std::vector<Kore::vec3> jointWorldPositions;

	// boneLength[0] == 0; boneLength[i] = jointWorldPositions[i] - joinWorldPositions[i-1]
	std::vector<float> boneLengths;

	Kore::vec3 rootWorldPosition;

	std::vector<EventListener> eventListeners;

	void fireEvent(DebugEvent event);

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

	void addListener(EventListener listener);
	//void removeListener(EventListener listener);
};


#endif // !_H_FABRIKSOLVER_
