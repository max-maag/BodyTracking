#include "StaticMesh.h"
#include "StaticMeshBuilder.h"

namespace utils {
	const std::shared_ptr<const StaticMesh> StaticMesh::CUBE =
		std::make_shared<const StaticMesh>(
			StaticMeshBuilder()
				.addQuad( // Front
					Kore::vec3(-1, 1, 1),
					Kore::vec3(-1, -1, 1),
					Kore::vec3(1, -1, 1),
					Kore::vec3(1, 1, 1)
				)
				.addQuad( // Right
					Kore::vec3(1,  1,  1),
					Kore::vec3(1, -1,  1),
					Kore::vec3(1, -1, -1),
					Kore::vec3(1,  1, -1)
				)
				.addQuad( // Back
					Kore::vec3( 1,  1, -1),
					Kore::vec3( 1, -1, -1),
					Kore::vec3(-1, -1, -1),
					Kore::vec3(-1,  1, -1)
				)
				.addQuad( // Left
					Kore::vec3(-1,  1, -1),
					Kore::vec3(-1, -1, -1),
					Kore::vec3(-1, -1,  1),
					Kore::vec3(-1,  1,  1)
				)
				.addQuad( // Top
					Kore::vec3(-1, 1, -1),
					Kore::vec3(-1, 1,  1),
					Kore::vec3( 1, 1,  1),
					Kore::vec3( 1, 1, -1)
				)
				.addQuad( // Bottom
					Kore::vec3( 1, -1,  1),
					Kore::vec3( 1, -1, -1),
					Kore::vec3(-1, -1, -1),
					Kore::vec3(-1, -1,  1)
				)
				.getMeshAndReset()
			);
}