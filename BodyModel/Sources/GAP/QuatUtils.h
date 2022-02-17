#pragma once

#include <Kore/Math/Vector.h>
#include <Kore/Math/Quaternion.h>

namespace Kore {
	namespace RotationUtility {
		Kore::Quaternion fromTo(Kore::vec3 from, Kore::vec3 to);
	}
}
