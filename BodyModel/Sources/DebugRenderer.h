#pragma once

#include <Kore/Math/Vector.h>
#include <Kore/Graphics4/Graphics.h>
#include <vector>
#include <memory>

class DebugRenderer
{
public:
	struct Vertex {
		Kore::vec3 position;
		Kore::vec3 normal;
	};

	struct Mesh {
		std::vector<Vertex> vertices;
		std::vector<int> indices;
	};

	static const Mesh CUBE;

private:
	std::unique_ptr<Kore::Graphics4::PipelineState> pipeline;
};

