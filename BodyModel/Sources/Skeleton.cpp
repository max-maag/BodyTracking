#include "Skeleton.h"

using namespace Kore;

Skeleton::Skeleton() = default;

Skeleton::Skeleton(const Skeleton& other): Skeleton() {
	*this = other;
}

Skeleton& Skeleton::operator=(const Skeleton& other) noexcept {
	bones = other.bones;

	// Fix parent pointers
	for (BoneNode& bone : bones) {
		size_t idxParent = bone.parent - &other.bones[0];

		// The root bone has a dummy bone as its parent that is not contained in bones and does not need to be fixed.
		if (idxParent < bones.size()) {
			bone.parent = &bones[idxParent];
		}
	}

	return *this;
}

Skeleton& Skeleton::operator=(Skeleton&& other) noexcept = default;

void Skeleton::reset() {
	for (int i = 0; i < bones.size(); ++i) {
		bones[i].transform = bones[i].bind;
		bones[i].local = bones[i].bind;
		bones[i].combined = bones[i].parent->combined * bones[i].local;
		bones[i].combinedInv = bones[i].combined.Invert();
		bones[i].finalTransform = bones[i].combined * bones[i].combinedInv;
		bones[i].rotation = Quaternion(0, 0, 0, 1);
	}
}


void Skeleton::setFixedPositionAndOrientation(int boneIndex, vec3 desPosition, Quaternion desRotation) {
	BoneNode& bone = getBoneWithIndex(boneIndex);

	bone.transform = mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z());
	bone.rotation = desRotation;
	bone.rotation.normalize();
	bone.calculateLocal();
}


void Skeleton::setFixedOrientation(int boneIndex, Quaternion desRotation) {
	BoneNode& bone = getBoneWithIndex(boneIndex);

	Quaternion localRot;
	RotationUtility::getOrientation(&bone.parent->combined, &localRot);
	bone.rotation = localRot.invert().rotated(desRotation);

	bone.rotation.normalize();
	bone.calculateLocal();
}

BoneNode& Skeleton::getBoneWithIndex(int boneIndex) {
	return bones[boneIndex - 1];
}