#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Base/Tools.h>
#include <Gui/Command.h>
#include <Gui/FileDialog.h>

#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoUnits.h>
// TODO: replace last link to be TransformerManipulator
#include "MeshUtility.h"
#include <Inventor/manips/SoTransformerManip.h>

using namespace Robot;
using Mesh::MeshObject;

MeshUtility::MeshUtility() {}

MeshUtility::~MeshUtility() {}

SoGroup *MeshUtility::generateMeshNode(const char *nodeName,
                                       const char *filePath,
                                       SoTransform *initTrans,
                                       float scaleFactor,
                                       Mesh_Material meshMaterial) {
  if (!strlen(filePath) || !strlen(nodeName))
    return nullptr;

  MeshObject m_meshObj;
  MeshCore::Material mat;

  try {
    if (m_meshObj.load(filePath, &mat)) {
      //  Generate Mesh
      auto coords = new SoCoordinate3();
      auto faces = new SoIndexedFaceSet();
      if (MeshUtility::createMeshFromKernal(m_meshObj, coords, faces)) {
        auto meshGroup = new SoSeparator();
        // Node Name
        meshGroup->setName(SbName(nodeName));
        // Transformation
        auto transNode = (initTrans != nullptr) ? initTrans : new SoTransform();
        std::string transNodeName = "TransitionOf";
        transNodeName.append(nodeName);
        transNode->setName(SbName(transNodeName.c_str()));
        // Materia
        SoMaterial *mat = new SoMaterial;
        mat->ambientColor.setValue(meshMaterial.ambientColor.data());
        mat->diffuseColor.setValue(meshMaterial.diffuseColor.data());
        mat->specularColor.setValue(meshMaterial.specularColor.data());
        mat->shininess = meshMaterial.shininess;
        std::string matNodeName = string("MatOf");
        matNodeName.append(nodeName);
        mat->setName(SbName(matNodeName.c_str()));
        // Scale
        float scale = (scaleFactor > 0) ? scaleFactor : 1.0;
        SoScale *meshScalePara = new SoScale();
        meshScalePara->scaleFactor.setValue(scale, scale, scale);
        meshScalePara->setName(SbName("Scale_Node"));

        meshGroup->addChild(transNode);
        meshGroup->addChild(meshScalePara);
        meshGroup->addChild(mat);
        meshGroup->addChild(coords);
        meshGroup->addChild(faces);
        return meshGroup;
      }
    }
    return nullptr;
  } catch (...) {
    std::cout << "Failed to open File" << std::endl;
    return nullptr;
  }
}

bool MeshUtility::createMeshFromKernal(Mesh::MeshObject const &target,
                                       SoCoordinate3 *coords,
                                       SoIndexedFaceSet *faces) {
  if (coords == nullptr || faces == nullptr)
    return false;
  const MeshCore::MeshKernel &obj_kernal = target.getKernel();
  // set the point coordinates
  const MeshCore::MeshPointArray &cP = obj_kernal.GetPoints();
  if (!obj_kernal.CountPoints())
    return false;
  coords->point.setNum(obj_kernal.CountPoints());
  SbVec3f *verts = coords->point.startEditing();
  unsigned long i = 0;
  for (MeshCore::MeshPointArray::_TConstIterator it = cP.begin();
       it != cP.end(); ++it, i++) {
    verts[i].setValue(it->x, it->y, it->z);
  }
  coords->point.finishEditing();

  // set the face indices
  unsigned long j = 0;
  const MeshCore::MeshFacetArray &cF = obj_kernal.GetFacets();
  if (!obj_kernal.CountFacets())
    return false;

  faces->coordIndex.setNum(4 * obj_kernal.CountFacets());
  int32_t *indices = faces->coordIndex.startEditing();
  for (MeshCore::MeshFacetArray::_TConstIterator it = cF.begin();
       it != cF.end(); ++it, j++) {
    for (int i = 0; i < 3; i++) {
      indices[4 * j + i] = it->_aulPoints[i];
    }
    indices[4 * j + 3] = SO_END_FACE_INDEX;
  }
  faces->coordIndex.finishEditing();
  return true;
}

bool MeshUtility::generateTmpSTLFile(const Part::PropertyPartShape &shape,
                                     std::string const &obj_Name, double accuracy) {
  Mesh::MeshObject m_MeshDataObj;
  std::vector<Base::Vector3d> vertices;
  std::vector<Data::ComplexGeoData::Facet> facets;
  auto data = shape.getComplexData();
  data->getFaces(vertices, facets, accuracy);
  if (vertices.empty() || facets.empty())
    return false;
  m_MeshDataObj.setFacets(facets, vertices);
  std::string fileName = tmpMeshFilePath + obj_Name + std::string(".stl");
  m_MeshDataObj.save(fileName.c_str(), MeshCore::MeshIO::BSTL, 0,
                     obj_Name.c_str());
  return true;
}
