#pragma once

//#include <RD_DataStructure/RD_STL/RD_String.h>
#include <memory>

#include <Mod/Mesh/App/Mesh.h>
#include <Mod/Mesh/App/MeshFeature.h>
#include <Mod/Part/App/PartFeature.h>

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>

#include "DS_Utility.h"

namespace Robot {
struct Mesh_Material {
  std::array<float, 3> ambientColor;
  std::array<float, 3> diffuseColor;
  std::array<float, 3> specularColor;
  float shininess;
  float transparent;
};
static Mesh_Material defaultMat = {
    {.33, .22, .27},
    {.78, .57, .11},
    {.99, .94, .81},
    0.28, 0.0};
static Mesh_Material collideMat = {
    {1.0, 0, 0},
    {1.0, 0, 0},
    {1.0, 0, 0},
    0.5, 0.85};

struct MeshUtility {
public:
  MeshUtility();
  ~MeshUtility();

  static SoGroup *generateMeshNode(const char *linkName, const char *filePath,
                                   SoTransform *linkTrans = nullptr,
                                   float scaleFactor = 1.0,
                                   Mesh_Material meshMaterial = defaultMat);

  static bool createMeshFromKernal(Mesh::MeshObject const &target,
                                   SoCoordinate3 *coords,
                                   SoIndexedFaceSet *faces);

  static bool generateTmpSTLFile(const Part::PropertyPartShape &shape,
                                 std::string const &obj_Name = "undefined",
                                 double accuracy = 0.8);
};
}
