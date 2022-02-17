#include "QuatUtils.h"

#include <cmath>

Kore::Quaternion Kore::RotationUtility::fromTo(Kore::vec3 from, Kore::vec3 to) {
	Kore::vec3 f(from);
	f.normalize();

	Kore::vec3 t(to);
	t.normalize();

	if (f == t) {
		return Kore::Quaternion();
	}
	else if (f == t * -1.0f) {
		Kore::vec3 ortho = Kore::vec3(1, 0, 0);
		if (fabsf(f.y()) < fabsf(f.x())) {
			ortho = Kore::vec3(0, 1, 0);
		}
		if (fabsf(f.z()) < fabs(f.y()) && fabs(f.z()) < fabsf(f.x())) {
			ortho = Kore::vec3(0, 0, 1);
		}

		Kore::vec3 axis = f.cross(ortho).normalize();
		return Kore::Quaternion(axis.x(), axis.y(), axis.z(), 0);
	}

	Kore::vec3 half = (f + t).normalize();
	Kore::vec3 axis = f.cross(half);

	return Kore::Quaternion(
		axis.x(),
		axis.y(),
		axis.z(),
		f.dot(half)
	);
}