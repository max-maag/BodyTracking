#pragma once

#include <memory>
#include <Kore/Math/Vector.h>
#include <Kore/Math/Quaternion.h>
#include <map>
#include <algorithm>

#include "IKEvaluator.h"
#include "RotationUtility.h"

struct BoneNode {
	char* boneName;
	int nodeIndex;
	int nodeDepth;
	BoneNode* parent;

	Kore::mat4 bind;
	Kore::mat4 transform; // local bone length in translation components?
	Kore::mat4 local; // parent end -> this end
	Kore::mat4 combined, combinedInv; // root -> this end
	Kore::mat4 finalTransform; // combined in world coordinates?

	Kore::Quaternion rotation;	// local rotation

	bool initialized = false;

	std::vector<Kore::mat4> aniTransformations;

	// Constraints
	Kore::vec3 axes;
	std::map<const char* const, float> constrain;	// <min, max>

	static const char* const xMin;
	static const char* const xMax;
	static const char* const yMin;
	static const char* const yMax;
	static const char* const zMin;
	static const char* const zMax;

	BoneNode() :
		transform(Kore::mat4::Identity()),
		local(Kore::mat4::Identity()),
		combined(Kore::mat4::Identity()),
		combinedInv(Kore::mat4::Identity()),
		finalTransform(Kore::mat4::Identity()),
		rotation(Kore::Quaternion(0, 0, 0, 1)),
		axes(Kore::vec3(0, 0, 0))
	{}

	Kore::vec3 getPosition() const {
		Kore::vec3 result;

		Kore::vec4 pos = combined * Kore::vec4(0, 0, 0, 1);
		pos *= 1.0 / pos.w();

		result.x() = pos.x();
		result.y() = pos.y();
		result.z() = pos.z();

		return result;
	}

	Kore::Quaternion getOrientation() const {
		Kore::Quaternion result;
		Kore::RotationUtility::getOrientation(&combined, &result);

		return result;
	}

	void update() {
		if (parent->initialized)
			combined = parent->combined * local;
	}

	void initialize() {
		update();

		if (!initialized) {
			initialized = true;
			combinedInv = combined.Invert();
		}

		finalTransform = combined * combinedInv;
	}

	void calculateLocal() {
		local = transform * rotation.matrix().Transpose();
	}

	void applyJointConstraints() {
		BoneNode* bone = this;
		while (bone->initialized) {
			Kore::vec3 axes = bone->axes;

			Kore::vec3 rot;
			Kore::RotationUtility::quatToEuler(&bone->rotation, &rot.x(), &rot.y(), &rot.z());

			float x = rot.x(), y = rot.y(), z = rot.z();

			if (axes.x() == 1.0) {
				x = std::clamp(x, bone->constrain[BoneNode::xMin], bone->constrain[BoneNode::xMax]);
			}

			if (axes.y() == 1.0) {
				y = std::clamp(y, bone->constrain[BoneNode::yMin], bone->constrain[BoneNode::yMax]);
			}

			if (axes.z() == 1.0) {
				z = std::clamp(z, bone->constrain[BoneNode::zMin], bone->constrain[BoneNode::zMax]);
			}

			Kore::RotationUtility::eulerToQuat(x, y, z, &bone->rotation);

			// bone->rotation = Kore::Quaternion((double) x, (double) y, (double) z, 1);
			bone->rotation.normalize();
			bone->calculateLocal();
			bone = bone->parent;
		}
	}
};

struct CompareBones {
	bool const operator()(BoneNode* bone1, BoneNode* bone2) const {
		return (bone1->nodeDepth) < (bone2->nodeDepth);
	}
};

class Skeleton
{
private:

public:
	Skeleton();
	Skeleton(const Skeleton& other);
	Skeleton& operator=(const Skeleton& other) noexcept;
	Skeleton& operator=(Skeleton&& other) noexcept;

	std::vector<BoneNode> bones;
	
	void reset();

	void setFixedPositionAndOrientation(int boneIndex, Kore::vec3 desPosition, Kore::Quaternion desRotation);
	void setFixedOrientation(int boneIndex, Kore::Quaternion desRotation);

	BoneNode& getBoneWithIndex(int boneIndex);
};

