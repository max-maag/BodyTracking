#include "StaticMeshRenderer.h"

#include <Kore/IO/FileReader.h>
#include <Kore/Graphics4/Shader.h>
#include <Kore/Log.h>
#include <functional>

#include "Utils.h"

using namespace Kore::Graphics4;
using std::placeholders::_1;

namespace {
	static const char* const NAME_CONST_MATRIX_VIEW = "V";
	static const char* const NAME_CONST_MATRIX_PROJECTION = "P";
	static const char* const NAME_CONST_NUM_LIGHTS = "numLights";
	static const char* const NAME_CONST_POS_LIGHTS_ARRAY = "lightPos";

	static const char* const NAME_SHADER_INPUT_MATRIX_MODEL = "M";
	static const char* const NAME_SHADER_INPUT_MATRIX_MODEL_INVERSE = "MInverse";

	static const char* const NAME_SHADER_INPUT_POSITION = "pos";
	static const char* const NAME_SHADER_INPUT_COLOR = "col";
	static const char* const NAME_SHADER_INPUT_NORMAL = "nor";

	static const auto DUMMY_ALLOCATOR = [](size_t size) { return nullptr; };
}

namespace utils {
	StaticMeshRenderer::StaticMeshRenderer():
		bufferVertexData(DUMMY_ALLOCATOR),
		bufferInstanceData(DUMMY_ALLOCATOR),
		bufferIndices(DUMMY_ALLOCATOR) {
	}

	void StaticMeshRenderer::initialize() {
		if (isInitialized) {
			Kore::log(Kore::LogLevel::Warning, "StaticMeshRenderer::initialize called on already initialized renderer.");
			return;
		}

		pipeline = std::make_unique<PipelineState>();

		loadShaders();
		setVertexInputStructures();


		bufferVertexData = G4VertexBuffer([this](size_t size) {
			return createVertexDataBuffer(size);
		});

		bufferInstanceData = G4VertexBuffer([this](size_t size) {
			return createInstanceDataBuffer(size);
		});

		bufferIndices = G4IndexBuffer(&StaticMeshRenderer::createIndexDataBuffer);


		pipeline->depthMode = ZCompareLess;
		pipeline->depthWrite = true;
		pipeline->blendSource = SourceAlpha;
		pipeline->blendDestination = InverseSourceAlpha;
		pipeline->alphaBlendSource = SourceAlpha;
		pipeline->alphaBlendDestination = InverseSourceAlpha;

		pipeline->compile();

		locationMatrixProjection = pipeline->getConstantLocation(NAME_CONST_MATRIX_PROJECTION);
		locationMatrixView = pipeline->getConstantLocation(NAME_CONST_MATRIX_VIEW);
		locationNumLights = pipeline->getConstantLocation(NAME_CONST_NUM_LIGHTS);
		locationPosLightsArray = pipeline->getConstantLocation(NAME_CONST_POS_LIGHTS_ARRAY);

		isInitialized = true;
	}

	Shader* StaticMeshRenderer::createShader(std::string filename, Kore::Graphics4::ShaderType shaderType) {
		Kore::FileReader reader(filename.c_str());
		return new Shader(reader.readAll(), reader.size(), shaderType);
	}

	void StaticMeshRenderer::loadShaders() {
		pipeline->vertexShader = createShader("static_shader.vert", Kore::Graphics4::ShaderType::VertexShader);
		pipeline->fragmentShader = createShader("static_shader.frag", Kore::Graphics4::ShaderType::FragmentShader);
	}

	void StaticMeshRenderer::setVertexInputStructures() {
		VertexStructure* structPerVertex = new VertexStructure();
		structPerVertex->instanced = false;
		structPerVertex->add(NAME_SHADER_INPUT_POSITION, Float3VertexData);
		structPerVertex->add(NAME_SHADER_INPUT_NORMAL, Float3VertexData);

		VertexStructure* structPerInstance = new VertexStructure();
		structPerInstance->instanced = true;
		structPerInstance->add(NAME_SHADER_INPUT_COLOR, Float4VertexData);
		structPerInstance->add(NAME_SHADER_INPUT_MATRIX_MODEL, Float4x4VertexData);
		structPerInstance->add(NAME_SHADER_INPUT_MATRIX_MODEL_INVERSE, Float4x4VertexData);

		pipeline->inputLayout[0] = structPerVertex;
		pipeline->inputLayout[1] = structPerInstance;
		pipeline->inputLayout[3] = nullptr;
	}

	std::shared_ptr<StaticMeshRenderer::Instance> StaticMeshRenderer::createInstance(std::shared_ptr<const StaticMesh> mesh, Kore::mat4 transform, Kore::Graphics1::Color color) {
		runtimeAssert(mesh != nullptr, "mesh is null");
		
		if (!meshData.contains(mesh)) {
			initMesh(mesh);
		}

		auto instance = std::make_shared<Instance>(color, transform);

		meshData[mesh].instances.push_back(instance);

		return instance;
	}

	void StaticMeshRenderer::render(Kore::mat4 projection, Kore::mat4 view, StaticMeshRenderer::LightsSetter setLights) {
		// Bad practice, but Kore shouldn't hang onto the pointer. At least not longer than the StaticMeshRenderer is alive.
		setPipeline(&*pipeline);

		VertexBuffer* vbuffers[] {
			bufferVertexData.getBuffer(),
			bufferInstanceData.getBuffer()
		};

		setVertexBuffers(vbuffers, 2);
		setIndexBuffer(*bufferIndices.getBuffer());

		setMatrix(locationMatrixProjection, projection);
		setMatrix(locationMatrixView, view);

		setLights(locationNumLights, locationPosLightsArray);


		for (const auto& [_, data] : meshData) {
			bufferInstanceData.clear();

			bufferInstanceData.ensureSize(data.instances.size());

			struct InstanceData {
				Kore::Graphics1::Color color = Kore::Graphics1::Color::White;
				Kore::mat4 transform;
				Kore::mat4 inverse;
			};

			std::vector<InstanceData> instanceData(data.instances.size());

			std::transform(data.instances.begin(), data.instances.end(), instanceData.begin(), [](auto instance) -> InstanceData {
				return {
					instance->color,
					instance->transform,
					instance->transform.Invert()
				};
			});

			bufferInstanceData.putMany(instanceData.begin(), instanceData.end());

			drawIndexedVerticesInstanced(data.instances.size(), data.indexStart, data.countIndices);
		}
		
	}

	void StaticMeshRenderer::initMesh(std::shared_ptr<const StaticMesh> mesh) {
		bufferVertexData.putMany(mesh->vertices.begin(), mesh->vertices.end());

		size_t indexStart = bufferIndices.calculateElementSize();

		bufferIndices.putMany(mesh->indices.begin(), mesh->indices.end());

		meshData[mesh] = {
			(int)indexStart,
			(int)(mesh->indices.size()),
			std::vector<std::shared_ptr<Instance>>()
		};
	}


	std::unique_ptr<VertexBuffer> StaticMeshRenderer::createVertexDataBuffer(size_t size) {
		return std::make_unique<VertexBuffer>(size, *(pipeline->inputLayout[0]));
	}


	std::unique_ptr<VertexBuffer> StaticMeshRenderer::createInstanceDataBuffer(size_t size) {
		return std::make_unique<VertexBuffer>(size, *(pipeline->inputLayout[1]), Kore::Graphics4::Usage::StaticUsage, 1);
	}

	std::unique_ptr<IndexBuffer> StaticMeshRenderer::createIndexDataBuffer(size_t size) {
		return std::make_unique<IndexBuffer>(size);
	}
}
