#pragma once

#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include <Kore/Graphics4/PipelineState.h>
#include <Kore/Graphics4/Shader.h>
#include <Kore/Graphics4/Graphics.h>

#include "StaticMesh.h"
#include "GraphicsBuffer.h"

namespace utils {
	class StaticMeshRenderer {
	public:
		typedef std::function<void(Kore::Graphics4::ConstantLocation, Kore::Graphics4::ConstantLocation)> LightsSetter;

		struct Instance {
			Kore::Graphics1::Color color = Kore::Graphics1::Color::Magenta;
			Kore::mat4 transform;
		};

		StaticMeshRenderer();

		// Adds a new instance of mesh to be rendered. The returned pointer can be used to change the instance.
		std::shared_ptr<Instance> createInstance(std::shared_ptr<const StaticMesh> mesh, Kore::mat4 transform, Kore::Graphics1::Color color);

		void initialize();

		// LightSetter is used to call LivingRoom::setLights. Ugly, but works.
		void render(Kore::mat4 perspective, Kore::mat4 view, LightsSetter setLights);

	private:
		static const int SIZE_BUFFER_INITIAL = 128;

		struct MeshData {
			int indexStart;
			int countIndices;
			std::vector<std::shared_ptr<Instance>> instances;
		};

		bool isInitialized = false;

		std::unique_ptr<Kore::Graphics4::PipelineState> pipeline;
		std::unordered_map<std::shared_ptr<const StaticMesh>, MeshData> meshData;

		G4VertexBuffer bufferVertexData;

		G4VertexBuffer bufferInstanceData;

		G4IndexBuffer bufferIndices;

		Kore::Graphics4::Shader* createShader(std::string filename, Kore::Graphics4::ShaderType shaderType);

		void loadShaders();
		void setVertexInputStructures();
		void initMesh(std::shared_ptr<const StaticMesh> mesh);

		std::unique_ptr<Kore::Graphics4::VertexBuffer> createVertexDataBuffer(size_t size);
		std::unique_ptr<Kore::Graphics4::VertexBuffer> createInstanceDataBuffer(size_t size);
		static std::unique_ptr<Kore::Graphics4::IndexBuffer> createIndexDataBuffer(size_t size);

		Kore::Graphics4::ConstantLocation locationMatrixProjection;
		Kore::Graphics4::ConstantLocation locationMatrixView;

		Kore::Graphics4::ConstantLocation locationNumLights;
		Kore::Graphics4::ConstantLocation locationPosLightsArray;
	};
}
