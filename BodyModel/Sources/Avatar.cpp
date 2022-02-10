#include "pch.h"
#include "Avatar.h"
#include "RotationUtility.h"
#include "JacobianIK.h"

#include <algorithm>

using namespace Kore;
using namespace Kore::Graphics4;

Avatar::Avatar(const char* meshFile, const char* textureFile, const Kore::Graphics4::VertexStructure& structure, std::shared_ptr<IKSolver> ikSolver, float scale) : MeshObject(meshFile, textureFile, structure, scale) {
	this->ikSolver = ikSolver;
	
	// Update bones
	for (int i = 0; i < bones.size(); ++i) {
		bones[i]->initialize();
	}

	
	// Get the highest position
	BoneNode* head = getBoneWithIndex(headBoneIndex);
	Kore::vec4 position = head->combined * Kore::vec4(0, 0, 0, 1);
	position *= 1.0/position.w();
	currentHeight = position.z();

	setJointConstraints();
}

void Avatar::animate(TextureUnit tex) {
	// Update bones
	for (int i = 0; i < bones.size(); ++i) {
		bones[i]->initialize();
	}
	
	for(int j = 0; j < meshesCount; ++j) {
		int currentBoneIndex = 0;	// Iterate over BoneCountArray
		
		Mesh* mesh = meshes[j];
		
		// Mesh Vertex Buffer
		float* vertices = vertexBuffers[j]->lock();
		for (int i = 0; i < mesh->numVertices; ++i) {
			vec4 startPos(0, 0, 0, 1);
			vec4 startNormal(0, 0, 0, 1);
			
			// For each vertex belonging to a mesh, the bone count array specifies the number of bones the influence the vertex
			int numOfBones = mesh->boneCountArray[i];
			
			float totalJointsWeight = 0;
			for (int b = 0; b < numOfBones; ++b) {
				vec4 posVec(mesh->vertices[i * 3 + 0], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2], 1);
				vec4 norVec(mesh->normals[i * 3 + 0], mesh->normals[i * 3 + 1], mesh->normals[i * 3 + 2], 1);
				
				int index = mesh->boneIndices[currentBoneIndex] + 2;
				BoneNode* bone = getBoneWithIndex(index);
				float boneWeight = mesh->boneWeight[currentBoneIndex];
				totalJointsWeight += boneWeight;
				
				startPos += (bone->finalTransform * posVec) * boneWeight;
				startNormal += (bone->finalTransform * norVec) * boneWeight;
				
				currentBoneIndex ++;
			}
			
			// position
			vertices[i * 8 + 0] = startPos.x() * scale;
			vertices[i * 8 + 1] = startPos.y() * scale;
			vertices[i * 8 + 2] = startPos.z() * scale;
			// texCoord
			vertices[i * 8 + 3] = mesh->texcoord[i * 2 + 0];
			vertices[i * 8 + 4] = 1.0f - mesh->texcoord[i * 2 + 1];
			// normal
			vertices[i * 8 + 5] = startNormal.x();
			vertices[i * 8 + 6] = startNormal.y();
			vertices[i * 8 + 7] = startNormal.z();
			
			//log(Info, "%f %f %f %f %f %f %f %f", vertices[i * 8 + 0], vertices[i * 8 + 1], vertices[i * 8 + 2], vertices[i * 8 + 3], vertices[i * 8 + 4], vertices[i * 8 + 5], vertices[i * 8 + 6], vertices[i * 8 + 7]);
		}
		vertexBuffers[j]->unlock();
		
		Texture* image = images[j];
		
		Graphics4::setTexture(tex, image);
		Graphics4::setVertexBuffer(*vertexBuffers[j]);
		Graphics4::setIndexBuffer(*indexBuffers[j]);
		Graphics4::drawIndexedVertices();
	}
}

void Avatar::setDesiredPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation, IKEvaluator* ikEvaluator) {
	BoneNode* bone = getBoneWithIndex(boneIndex);
	
	ikSolver->solve(bone, desPosition, desRotation, ikEvaluator);
}

void Avatar::setFixedPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation) {
	BoneNode* bone = getBoneWithIndex(boneIndex);
	
	bone->transform = mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z());
	bone->rotation = desRotation;
	bone->rotation.normalize();
	bone->local = bone->transform * bone->rotation.matrix().Transpose();
}

void Avatar::setFixedOrientation(int boneIndex, Kore::Quaternion desRotation) {
	BoneNode* bone = getBoneWithIndex(boneIndex);
	
	Kore::Quaternion localRot;
	Kore::RotationUtility::getOrientation(&bone->parent->combined, &localRot);
	bone->rotation = localRot.invert().rotated(desRotation);
	
	//bone->transform = mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z());
	//bone->rotation = desRotation;
	
	bone->rotation.normalize();
	bone->local = bone->transform * bone->rotation.matrix().Transpose();
}

BoneNode* Avatar::getBoneWithIndex(int boneIndex) const {
	BoneNode* bone = bones[boneIndex - 1];
	return bone;
}

void Avatar::resetPositionAndRotation() {
	for (int i = 0; i < bones.size(); ++i) {
		bones[i]->transform = bones[i]->bind;
		bones[i]->local = bones[i]->bind;
		bones[i]->combined = bones[i]->parent->combined * bones[i]->local;
		bones[i]->combinedInv = bones[i]->combined.Invert();
		bones[i]->finalTransform = bones[i]->combined * bones[i]->combinedInv;
		bones[i]->rotation = Kore::Quaternion(0, 0, 0, 1);
	}
}

float Avatar::getHeight() const {
	return currentHeight;
}

std::shared_ptr<IKSolver> Avatar::getIkSolver() {
	return ikSolver;
}

void Avatar::setIkSolver(std::shared_ptr<IKSolver> solver) {
	this->ikSolver = solver;
}


void Avatar::setJointConstraints() {
	BoneNode* nodeLeft;
	BoneNode* nodeRight;

	float tolerance = RotationUtility::getRadians(15);

	// Head
	nodeLeft = bones[headBoneIndex - 1];
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
	nodeLeft = bones[spineBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 1);

	nodeLeft->constrain[BoneNode::xMin] = -RotationUtility::getRadians(20) - tolerance;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(0) + tolerance;

	nodeLeft->constrain[BoneNode::zMin] = -RotationUtility::getRadians(35) - tolerance;
	nodeLeft->constrain[BoneNode::zMax] = RotationUtility::getRadians(35) + tolerance;

	log(LogLevel::Info, "Spine");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));
	log(LogLevel::Info, "zmin %f zmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::zMax]));

	// Upperarm
	nodeLeft = bones[leftArmBoneIndex - 1];
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

	nodeRight = bones[rightArmBoneIndex - 1];
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
	nodeLeft = bones[leftForeArmBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 0);

	nodeLeft->constrain[BoneNode::xMin] = 0;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(140) + tolerance;

	log(LogLevel::Info, "Elbow Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));

	nodeRight = bones[rightForeArmBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	log(LogLevel::Info, "Elbow Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));

	// Hand
	nodeLeft = bones[leftHandBoneIndex - 1];
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

	nodeRight = bones[rightHandBoneIndex - 1];
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
	nodeLeft = bones[leftUpLegBoneIndex - 1];
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

	nodeRight = bones[rightUpLegBoneIndex - 1];
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
	nodeLeft = bones[leftLegBoneIndex - 1];
	nodeLeft->axes = Kore::vec3(1, 0, 0);

	nodeLeft->constrain[BoneNode::xMin] = 0;
	nodeLeft->constrain[BoneNode::xMax] = RotationUtility::getRadians(150) + tolerance;

	log(LogLevel::Info, "Knee Left");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeLeft->constrain[BoneNode::xMax]));

	nodeRight = bones[rightLegBoneIndex - 1];
	nodeRight->axes = nodeLeft->axes;

	nodeRight->constrain[BoneNode::xMin] = nodeLeft->constrain[BoneNode::xMin];
	nodeRight->constrain[BoneNode::xMax] = nodeLeft->constrain[BoneNode::xMax];

	log(LogLevel::Info, "Knee Right");
	log(LogLevel::Info, "xmin %f xmax %f", RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMin]), RotationUtility::getDegree(nodeRight->constrain[BoneNode::xMax]));

	for (BoneNode* bone : bones) {
		if (bone->constrain[BoneNode::xMin] > bone->constrain[BoneNode::xMax]) {
			std::swap(bone->constrain[BoneNode::xMin], bone->constrain[BoneNode::xMax]);
		}

		if (bone->constrain[BoneNode::yMin] > bone->constrain[BoneNode::yMax]) {
			std::swap(bone->constrain[BoneNode::yMin], bone->constrain[BoneNode::yMax]);
		}

		if (bone->constrain[BoneNode::zMin] > bone->constrain[BoneNode::zMax]) {
			std::swap(bone->constrain[BoneNode::zMin], bone->constrain[BoneNode::zMax]);
		}
	}
}