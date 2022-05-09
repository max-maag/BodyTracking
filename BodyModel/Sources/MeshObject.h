#pragma once

#include "OpenGEX/OpenGEX.h"
#include "RotationUtility.h"
#include "Skeleton.h"

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
	Skeleton* skeleton;
	std::vector<Light*> lights;
	
	Material* findMaterialWithIndex(const int index);
	
private:
	void LoadObj(const char* filename);
	
	void ConvertObjects(const Structure& structure);
	Mesh* ConvertGeometryObject(const OGEX::GeometryObjectStructure& structure);
	Mesh* ConvertMesh(const OGEX::MeshStructure& structure, const char* geometryName);
	
	Material* ConvertMaterial(const OGEX::MaterialStructure& structure);
	
	void ConvertNodes(const Structure& structure, BoneNode* parentNode, std::map<BoneNode*, std::vector<BoneNode*>>& boneChildren, std::vector<BoneNode*>& boneOrder);
	Geometry* ConvertGeometryNode(const OGEX::GeometryNodeStructure& structure);
	BoneNode* ConvertBoneNode(const OGEX::BoneNodeStructure& structure);
	
	Light* ConvertLightNode(const OGEX::LightNodeStructure& structure);
};
