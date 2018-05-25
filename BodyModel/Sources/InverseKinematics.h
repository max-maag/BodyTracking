#pragma once

#include <Kore/Math/Quaternion.h>

#include <vector>

struct BoneNode;

class InverseKinematics {
	
public:
	InverseKinematics(std::vector<BoneNode*> bones, int maxSteps);
    bool inverseKinematics(BoneNode* targetBone, Kore::vec4 desiredPosition, Kore::Quaternion desiredRotation);
	
	float getAverageIter();
	
private:
	int boneCount;
	int maxSteps;
	float maxError;
	int rootIndex;
	bool clamp;
	
	std::vector<BoneNode*> bones;
	
	static const int jacDim = 6;
	static const int maxBones = 3;
    
	typedef Kore::Matrix<jacDim, maxBones, float> mat6x;
	typedef Kore::Vector<float, jacDim> vec6;
	typedef Kore::Matrix<jacDim, jacDim, float> mat6x6;
	
	void setJointConstraints();
	void applyChanges(std::vector<float> theta, BoneNode* targetBone);
	void updateBonePosition(BoneNode* targetBone);
	void applyJointConstraints(BoneNode* targetBone);
	bool clampValue(float minVal, float maxVal, float* value);
	
	int totalNum;
	int sumIter;
};
