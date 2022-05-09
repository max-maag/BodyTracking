#pragma once

#include <memory>
#include "MeshObject.h"
#include "IKSolver.h"


class Avatar : public MeshObject {
	
private:
	float currentHeight;

	void setJointConstraints();

public:
	Avatar(const char* meshFile, const char* textureFile, const Kore::Graphics4::VertexStructure& structure, float scale = 1.0f);
	
	void animate(Kore::Graphics4::TextureUnit tex);
	
	//void setDesiredPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation, IKEvaluator* ikEvaluator);


	
	float getHeight() const;
};
