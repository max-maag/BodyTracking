#pragma once

#include "OpenGEX/OpenGEX.h"
#include "RotationUtility.h"

#include <Kore/Graphics4/Graphics.h>
#include <Kore/Math/Quaternion.h>

#include <vector>
#include <map>
#include <algorithm>

struct Mesh {
	int numFaces;
	int numVertices;
	int numUVs;
	int numNormals;
	
	float* vertices;
	int* indices;
	float* normals;
	float* texcoord;
	
	// Skin
	unsigned_int16* boneCountArray;
	int boneCount;
	unsigned_int16* boneIndices;
	int boneIndexCount;
	float* boneWeight;
	int weightCount;
	
	unsigned int meshIndex;
};

struct CompareMesh {
	bool const operator()(Mesh* mesh1, Mesh* mesh2) const {
		return (mesh1->meshIndex) < (mesh2->meshIndex);
	}
};

struct Geometry {
	Kore::mat4 transform;
	const char* name;
	const char* objectRef;
	const char* materialRef;
	unsigned int materialIndex;
	unsigned int geometryIndex;
};

struct CompareGeometry {
	bool const operator()(Geometry* geo1, Geometry* geo2) const {
		return (geo1->geometryIndex) < (geo2->geometryIndex);
	}
};

struct Light {
	Kore::vec4 position;
	const char* name;
	int type;
};

struct Material {
	char* materialName;
	char* textureName;
	unsigned int materialIndex;
	
	int texScaleX;
	int texScaleY;
	
	Kore::vec3 diffuse;
	Kore::vec3 specular;
	float specular_power;
};

struct CompareMaterials {
	bool const operator()(Material* material1, Material* material2) const {
		return (material1->materialIndex) < (material2->materialIndex);
	}
};

struct BoneNode {
	char* boneName;
	int nodeIndex;
	int nodeDepth;
	BoneNode* parent;
	
	Kore::mat4 bind;
	Kore::mat4 transform;
	Kore::mat4 local;
	Kore::mat4 combined, combinedInv;
	Kore::mat4 finalTransform;
	
	Kore::Quaternion rotation;	// local rotation
	
	bool initialized = false;
	
	std::vector<Kore::mat4> aniTransformations;
	
	// Constraints
	Kore::vec3 axes;
	std::map<const char* const, float> constrain;	// <min, max>

	static const char* const xMin;
	static const char* const xMax;
	static const char* const yMin;
	static const char* const yMax;
	static const char* const zMin;
	static const char* const zMax;
	
	BoneNode() :
		transform(Kore::mat4::Identity()),
		local(Kore::mat4::Identity()),
		combined(Kore::mat4::Identity()),
		combinedInv(Kore::mat4::Identity()),
		finalTransform(Kore::mat4::Identity()),
		rotation(Kore::Quaternion(0, 0, 0, 1)),
		axes(Kore::vec3(0, 0, 0))
	{}
	
	Kore::vec3 getPosition() {
		Kore::vec3 result;
		
		Kore::vec4 pos = combined * Kore::vec4(0, 0, 0, 1);
		pos *= 1.0 / pos.w();
		
		result.x() = pos.x();
		result.y() = pos.y();
		result.z() = pos.z();
		
		return result;
	}
	
	Kore::Quaternion getOrientation() {
		Kore::Quaternion result;
		Kore::RotationUtility::getOrientation(&combined, &result);
		
		return result;
	}

	void update() {
		if (parent->initialized)
			combined = parent->combined * local;
	}

	void initialize() {
		update();

		if (!initialized) {
			initialized = true;
			combinedInv = combined.Invert();
		}

		finalTransform = combined * combinedInv;
	}

	void applyJointConstraints() {
		BoneNode* bone = this;
		while (bone->initialized) {
			Kore::vec3 axes = bone->axes;

			Kore::vec3 rot;
			Kore::RotationUtility::quatToEuler(&bone->rotation, &rot.x(), &rot.y(), &rot.z());

			float x = rot.x(), y = rot.y(), z = rot.z();

			if (axes.x() == 1.0) {
				x = std::clamp(x, bone->constrain[BoneNode::xMin], bone->constrain[BoneNode::xMax]);
			}

			if (axes.y() == 1.0) {
				y = std::clamp(y, bone->constrain[BoneNode::yMin], bone->constrain[BoneNode::yMax]);
			}

			if (axes.z() == 1.0) {
				z = std::clamp(z, bone->constrain[BoneNode::zMin], bone->constrain[BoneNode::zMax]);
			}

			Kore::RotationUtility::eulerToQuat(x, y, z, &bone->rotation);

			// bone->rotation = Kore::Quaternion((double) x, (double) y, (double) z, 1);
			bone->rotation.normalize();
			bone->local = bone->transform * bone->rotation.matrix().Transpose();
			bone = bone->parent;
		}
	}
};

struct CompareBones {
	bool const operator()(BoneNode* bone1, BoneNode* bone2) const {
		return (bone1->nodeDepth) < (bone2->nodeDepth);
	}
};

class MeshObject {
public:
	MeshObject(const char* meshFile, const char* textureFile, const Kore::Graphics4::VertexStructure& structure, float scale = 1.0f);
	void render(Kore::Graphics4::TextureUnit tex);
	
	void setScale(float scaleFactor);
	Kore::mat4 M;
	Kore::mat4 Mmirror;
	
	long meshesCount;
	float scale;
	const Kore::Graphics4::VertexStructure& structure;
	Kore::Graphics4::VertexBuffer** vertexBuffers;
	Kore::Graphics4::IndexBuffer** indexBuffers;
	
	Kore::Graphics4::Texture** images;
	
	const char* textureDir;
	std::vector<Mesh*> meshes;
	std::vector<Geometry*> geometries;
	std::vector<Material*> materials;
	std::vector<BoneNode*> bones;
	std::vector<BoneNode*> children;
	std::vector<Light*> lights;
	
	Material* findMaterialWithIndex(const int index);
	
private:
	void LoadObj(const char* filename);
	
	void ConvertObjects(const Structure& structure);
	Mesh* ConvertGeometryObject(const OGEX::GeometryObjectStructure& structure);
	Mesh* ConvertMesh(const OGEX::MeshStructure& structure, const char* geometryName);
	
	Material* ConvertMaterial(const OGEX::MaterialStructure& structure);
	
	void ConvertNodes(const Structure& structure, BoneNode& parentNode);
	Geometry* ConvertGeometryNode(const OGEX::GeometryNodeStructure& structure);
	BoneNode* ConvertBoneNode(const OGEX::BoneNodeStructure& structure);
	
	Light* ConvertLightNode(const OGEX::LightNodeStructure& structure);
};
