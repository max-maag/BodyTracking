#include "pch.h"
#include "Avatar.h"
#include "RotationUtility.h"
#include "JacobianIK.h"

#include <algorithm>

using namespace Kore;
using namespace Kore::Graphics4;

Avatar::Avatar(const char* meshFile, const char* textureFile, const Kore::Graphics4::VertexStructure& structure, float scale) : MeshObject(meshFile, textureFile, structure, scale) {
	
	// skeleton->bones
	for (int i = 0; i < skeleton->bones.size(); ++i) {
		skeleton->bones[i].initialize();
	}

	
	// Get the highest position
	BoneNode& head = skeleton->getBoneWithIndex(headBoneIndex);
	Kore::vec4 position = head.combined * Kore::vec4(0, 0, 0, 1);
	position *= 1.0/position.w();
	currentHeight = position.z();

	setJointConstraints();
}

void Avatar::animate(TextureUnit tex) {
	// Update skeleton->bones
	for (int i = 0; i < skeleton->bones.size(); ++i) {
		skeleton->bones[i].initialize();
	}
	
	for(int idxMesh = 0; idxMesh < meshesCount; ++idxMesh) {
		int currentBoneIndex = 0;	// Iterate over BoneCountArray
		
		Mesh* mesh = meshes[idxMesh];
		
		// Mesh Vertex Buffer
		float* vertices = vertexBuffers[idxMesh]->lock();
		for (int idxVertex = 0; idxVertex < mesh->numVertices; ++idxVertex) {
			vec4 startPos(0, 0, 0, 1);
			vec4 startNormal(0, 0, 0, 1);
			
			// For each vertex belonging to a mesh, the bone count array specifies the number of skeleton->bones the influence the vertex
			int numOfBones = mesh->boneCountArray[idxVertex];
			
			float totalJointsWeight = 0;
			for (int b = 0; b < numOfBones; ++b) {
				vec4 posVec(mesh->vertices[idxVertex * 3 + 0], mesh->vertices[idxVertex * 3 + 1], mesh->vertices[idxVertex * 3 + 2], 1);
				vec4 norVec(mesh->normals[idxVertex * 3 + 0], mesh->normals[idxVertex * 3 + 1], mesh->normals[idxVertex * 3 + 2], 1);
				
				int index = mesh->boneIndices[currentBoneIndex] + 2;
				BoneNode& bone = skeleton->getBoneWithIndex(index);
				float boneWeight = mesh->boneWeight[currentBoneIndex];
				totalJointsWeight += boneWeight;
				
				startPos += (bone.finalTransform * posVec) * boneWeight;
				startNormal += (bone.finalTransform * norVec) * boneWeight;
				
				currentBoneIndex ++;
			}
			
			// position
			vertices[idxVertex * 8 + 0] = startPos.x() * scale;
			vertices[idxVertex * 8 + 1] = startPos.y() * scale;
			vertices[idxVertex * 8 + 2] = startPos.z() * scale;
			// texCoord
			vertices[idxVertex * 8 + 3] = mesh->texcoord[idxVertex * 2 + 0];
			vertices[idxVertex * 8 + 4] = 1.0f - mesh->texcoord[idxVertex * 2 + 1];
			// normal
			vertices[idxVertex * 8 + 5] = startNormal.x();
			vertices[idxVertex * 8 + 6] = startNormal.y();
			vertices[idxVertex * 8 + 7] = startNormal.z();
			
			//log(Info, "%f %f %f %f %f %f %f %f", vertices[i * 8 + 0], vertices[i * 8 + 1], vertices[i * 8 + 2], vertices[i * 8 + 3], vertices[i * 8 + 4], vertices[i * 8 + 5], vertices[i * 8 + 6], vertices[i * 8 + 7]);
		}
		vertexBuffers[idxMesh]->unlock();
		
		Texture* image = images[idxMesh];
		
		Graphics4::setTexture(tex, image);
		Graphics4::setVertexBuffer(*vertexBuffers[idxMesh]);
		Graphics4::setIndexBuffer(*indexBuffers[idxMesh]);
		Graphics4::drawIndexedVertices();
	}
}
/*
void Avatar::setDesiredPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation, IKEvaluator* ikEvaluator) {
	BoneNode& bone = getBoneWithIndex(boneIndex);
	
	ikSolver->solve(bone, desPosition, desRotation, ikEvaluator);
}
*/


float Avatar::getHeight() const {
	return currentHeight;
}


void Avatar::setJointConstraints() {
	BoneNode* nodeLeft;
	BoneNode* nodeRight;

	float tolerance = RotationUtility::getRadians(15);

	// Head
	nodeLeft = &skeleton->bones[headBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 1, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(40) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(40) + tolerance;

	nodeLeft->constrain[BoneNode::yMin] = -RotationUtility::getRadians(90) - tolerance;
	nodeLeft->constrain[BoneNode::yMax] = RotationUtility::getRadians(90) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(60) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(60) + tolerance;

	log(LogLevel::Info, "Head");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	// Neck
	nodeLeft = &skeleton->bones[spineBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(20) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(0) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(35) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(35) + tolerance;

	log(LogLevel::Info, "Spine");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	// Upperarm
	nodeLeft = &skeleton->bones[leftArmBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 1, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(50) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(130) + tolerance;

	nodeLeft->constrain[BoneNode::yMin] = -RotationUtility::getRadians(90) - tolerance;
	nodeLeft->constrain[BoneNode::yMax] = RotationUtility::getRadians(90) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(90) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(90) + tolerance;

	log(LogLevel::Info, "Shoulder Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	nodeRight = &skeleton->bones[rightArmBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	nodeRight->constrain[BoneNode::yMin] = -nodeLeft->constrain[BoneNode::yMin];
	nodeRight->constrain[BoneNode::yMax] = -nodeLeft->constrain[BoneNode::yMax];

	nodeRight->constrain[BoneNode::zMin] = -nodeLeft->constrain[BoneNode::zMin];
	nodeRight->constrain[BoneNode::zMax] = -nodeLeft->constrain[BoneNode::zMax];

	log(LogLevel::Info, "Shoulder Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMax]));

	// Forearm
	nodeLeft = &skeleton->bones[leftForeArmBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 0);

	nodeLeft->constrain[BoneNode::xMin] = 0;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(140) + tolerance;

	log(LogLevel::Info, "Elbow Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));

	nodeRight = &skeleton->bones[rightForeArmBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	log(LogLevel::Info, "Elbow Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));

	// Hand
	nodeLeft = &skeleton->bones[leftHandBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 1, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(30) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(30) + tolerance;

	nodeLeft->constrain[BoneNode::yMin] = -RotationUtility::getRadians(80) - tolerance;
	nodeLeft->constrain[BoneNode::yMax] = RotationUtility::getRadians(80) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(60) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(60) + tolerance;

	log(LogLevel::Info, "Hand Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	nodeRight = &skeleton->bones[rightHandBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	nodeRight->constrain[BoneNode::yMin] = -nodeLeft->constrain[BoneNode::yMin];
	nodeRight->constrain[BoneNode::yMax] = -nodeLeft->constrain[BoneNode::yMax];

	nodeRight->constrain[BoneNode::zMin] = -nodeLeft->constrain[BoneNode::zMin];
	nodeRight->constrain[BoneNode::zMax] = -nodeLeft->constrain[BoneNode::zMax];

	log(LogLevel::Info, "Hand Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMax]));

	// Thigh
	nodeLeft = &skeleton->bones[leftUpLegBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 1, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(110) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(30) + tolerance;

	nodeLeft->constrain[BoneNode::yMin] = -RotationUtility::getRadians(50) - tolerance;
	nodeLeft->constrain[BoneNode::yMax] = RotationUtility::getRadians(40) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(40) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(0) + tolerance;

	log(LogLevel::Info, "Upper Leg Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	nodeRight = &skeleton->bones[rightUpLegBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	nodeRight->constrain[BoneNode::yMin] = -nodeLeft->constrain[BoneNode::yMin];
	nodeRight->constrain[BoneNode::yMax] = -nodeLeft->constrain[BoneNode::yMax];

	nodeRight->constrain[BoneNode::zMin] = -nodeLeft->constrain[BoneNode::zMin];
	nodeRight->constrain[BoneNode::zMax] = -nodeLeft->constrain[BoneNode::zMax];

	log(LogLevel::Info, "Upper Leg Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "ymin %f ymax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::yMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::zMax]));

	// Calf
	nodeLeft = &skeleton->bones[leftLegBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 0);

	nodeLeft->constrain[BoneNode::xMin] = 0;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(150) + tolerance;

	log(LogLevel::Info, "Knee Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));

	nodeRight = &skeleton->bones[rightLegBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	log(LogLevel::Info, "Knee Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));

	for (BoneNode& bone : skeleton->bones) {
		if (bone.constrain[BoneNode::xMin] > bone.constrain[BoneNode::xMax]) {
			std::swap(bone.constrain[BoneNode::xMin], bone.constrain[BoneNode::xMax]);
		}

		if (bone.constrain[BoneNode::yMin] > bone.constrain[BoneNode::yMax]) {
			std::swap(bone.constrain[BoneNode::yMin], bone.constrain[BoneNode::yMax]);
		}

		if (bone.constrain[BoneNode::zMin] > bone.constrain[BoneNode::zMax]) {
			std::swap(bone.constrain[BoneNode::zMin], bone.constrain[BoneNode::zMax]);
		}
	}
}