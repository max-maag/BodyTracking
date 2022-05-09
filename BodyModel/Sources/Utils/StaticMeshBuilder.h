#pragma once

#include <Kore/Math/Vector.h>
#include <Kore/Graphics1/Color.h>
#include "StaticMesh.h"

namespace utils {
	class StaticMeshBuilder {
	private:
		StaticMesh mesh;

	public:
		StaticMeshBuilder& addTriangle(Kore::vec3 v1, Kore::vec3 v2, Kore::vec3 v3);
		StaticMeshBuilder& addQuad(Kore::vec3 v1, Kore::vec3 v2, Kore::vec3 v3, Kore::vec3 v4);
		StaticMesh getMeshAndReset();
	};
}
