#pragma once

#include <Kore/Math/Vector.h>
#include <Kore/Math/Quaternion.h>

namespace Kore {
	
	namespace RotationUtility {
		void eulerToQuat(const float roll, const float pitch, const float yaw, Kore::Quaternion* quat);
		void quatToEuler(const Kore::Quaternion* quat, float* roll, float* pitch, float* yaw);
		float getRadians(float degree);
		float getDegree(float rad);
		void getOrientation(const Kore::mat4* m, Kore::Quaternion* orientation);
		Kore::vec3 rotate(Kore::Quaternion q, Kore::vec3 v);
		Kore::Quaternion getRotationFromTo(Kore::vec3 v1, Kore::vec3 v2);
	}
}
