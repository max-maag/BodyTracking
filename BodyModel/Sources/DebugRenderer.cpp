#include "DebugRenderer.h"

/*
struct Mesh {
		std::vector<Vertex> vertices;
		std::vector<int> indices;
	};

*/
const DebugRenderer::Mesh DebugRenderer::CUBE {
	{
		// Front face
		{ // 0
			{-1,  1,  1},
			{ 0,  0,  1}
		},
		{ // 1
			{-1, -1,  1},
			{ 0,  0,  1}
		},
		{ // 2
			{ 1,  1,  1},
			{ 0,  0,  1}
		},
		{ // 3
			{ 1, -1,  1},
			{ 0,  0,  1}
		},

		// Right face
		{ // 4
			{ 1,  1,  1},
			{ 1,  0,  0}
		}
	},
	{
	}
};
