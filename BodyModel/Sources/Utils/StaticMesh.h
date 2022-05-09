#pragma once

#include <vector>
#include <memory>
#include <Kore/Math/Vector.h>
#include <Kore/Graphics1/Color.h>

namespace utils {
	struct StaticMesh {
		struct Vertex {
			Kore::vec3 position;
			Kore::vec3 normal;
		};

		std::vector<Vertex> vertices;
		std::vector<int> indices;

		// Cube centered around origin, side length = 2
		static const std::shared_ptr<const StaticMesh> CUBE;
	};
}