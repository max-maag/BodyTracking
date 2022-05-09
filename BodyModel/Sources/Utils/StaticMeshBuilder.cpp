#include "StaticMeshBuilder.h"

namespace utils {
	StaticMeshBuilder& StaticMeshBuilder::addTriangle(Kore::vec3 v1, Kore::vec3 v2, Kore::vec3 v3) {
		Kore::vec3 normal = (v2 - v1).cross(v3 - v1).normalize();

		int index = (int) mesh.vertices.size();

		mesh.vertices.push_back({ v1, normal });
		mesh.vertices.push_back({ v2, normal });
		mesh.vertices.push_back({ v3, normal });

		mesh.indices.push_back(index);
		mesh.indices.push_back(index+1);
		mesh.indices.push_back(index+2);

		return *this;
	}

	StaticMeshBuilder& StaticMeshBuilder::addQuad(Kore::vec3 v1, Kore::vec3 v2, Kore::vec3 v3, Kore::vec3 v4) {
		int index = (int) mesh.vertices.size();

		addTriangle(v1, v2, v3);

		mesh.vertices.push_back({ v4, mesh.vertices.back().normal });

		mesh.indices.push_back(index);
		mesh.indices.push_back(index+2);
		mesh.indices.push_back(index+3);

		return *this;
	}

	StaticMesh StaticMeshBuilder::getMeshAndReset() {
		return std::move(mesh);
	}
}