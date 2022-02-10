#pragma once

#include <memory>
#include "MeshObject.h"
#include "IKSolver.h"


class Avatar : public MeshObject {
	
private:
	std::shared_ptr<IKSolver> ikSolver;
	float currentHeight;

	void setJointConstraints();

public:
	Avatar(const char* meshFile, const char* textureFile, const Kore::Graphics4::VertexStructure& structure, std::shared_ptr<IKSolver> ikSolver, float scale = 1.0f);
	
	void animate(Kore::Graphics4::TextureUnit tex);
	void setDesiredPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation, IKEvaluator* ikEvaluator);
	void setFixedPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation);
	void setFixedOrientation(int boneIndex, Kore::Quaternion desRotation);

	std::shared_ptr<IKSolver> getIkSolver();
	void setIkSolver(std::shared_ptr<IKSolver> solver);
	
	BoneNode* getBoneWithIndex(int index) const;
	
	void resetPositionAndRotation();
	
	float getHeight() const;
};
