#ifndef _H_FABRIKSOLVER_
#define _H_FABRIKSOLVER_

#include <vector>
#include "../IKSolver.h"
#include <unordered_map>
#include <functional>

class FABRIKSolver : public IKSolver {
public:
	enum class EventType {
		IterationStepComplete, IterationComplete, SolveComplete
	};

	using EventListener = std::function<void(EventType, std::unordered_map<const BoneNode*, Kore::vec3>)>;

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

	void fireEvent(EventType eventType);

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
