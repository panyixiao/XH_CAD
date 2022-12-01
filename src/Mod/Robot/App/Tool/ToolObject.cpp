// Created by Yixiao 20220424
#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "ToolObject.h"
#include "Mod/Robot/App/Utilites/MeshUtility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/App/Utilites/PartUtility.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"

#include <App/Property.h>
#include <Base/Console.h>
#include <Base/FileInfo.h>
#include <Base/Reader.h>
#include <Base/Writer.h>

using namespace App;
using namespace Robot;
//using namespace RD_CAD_Utility;

PROPERTY_SOURCE(Robot::ToolObject, Part::Feature)

Eigen::Affine3d toAffine3d(Base::Matrix4D const &mat) {
  Eigen::Affine3d aff;
  aff.setIdentity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      aff.linear().row(i)[j] = mat[i][j];
    }
  }
  for (int i = 0; i < 3; ++i) {
    aff.translation()[i] = mat[i][3] / 1000;
  }
  return aff;
}

Base::Vector3d rotateByEigenMatrix(const Eigen::Affine3d &mat,
                                   const Base::Vector3d &v_src) {
  Eigen::Vector3d v(v_src.x, v_src.y, v_src.z);
  Eigen::Vector3d res = mat * v;
  return Base::Vector3d(res[0], res[1], res[2]);
}

ToolObject::ToolObject() {
  ADD_PROPERTY_TYPE(CAD_File, (""), "File", Prop_None, "Path to Model File");
  ADD_PROPERTY_TYPE(FilePath, (""), "File", Prop_None, "Path to Tool File");
  ADD_PROPERTY_TYPE(ParentRobotName, (""), "Assemble", Prop_None, "Assembled Robot Name");
  ADD_PROPERTY_TYPE(Type, (0), "Property", Prop_None, "Tool Type");

  ADD_PROPERTY(Pose_Mount, (Base::Placement()));
  ADD_PROPERTY(Trans_O2M, (Base::Placement()));
  ADD_PROPERTY(Trans_M2T, (Base::Placement()));
  ADD_PROPERTY(Trans_T2F, (Base::Placement()));
  ADD_PROPERTY(setEdit, (false));
  CAD_File.setStatus(App::Property::Status::ReadOnly, true);
  FilePath.setStatus(App::Property::Status::ReadOnly, true);
  ParentRobotName.setStatus(App::Property::Status::ReadOnly, true);
  Placement.setStatus(App::Property::Status::Hidden, true);

  setEdit.setStatus(App::Property::Status::ReadOnly, true);

  ADD_PROPERTY_TYPE(LinkedEdgeFeature, (0), "", Prop_None, "Linked Edge Feature for attachment and tip setup");
  LinkedEdgeFeature.setStatus(App::Property::Status::Hidden, true);
  ADD_PROPERTY_TYPE(LinkedFaceFeature, (0), "", Prop_None, "Linked Face Feature for attachment and tip setup");
  LinkedFaceFeature.setStatus(App::Property::Status::Hidden, true);
}

ToolObject::~ToolObject() {}

bool ToolObject::loadTool(App::Document *pcDoc, const string &filePath) {
  if (pcDoc == nullptr || filePath.empty())
    return false;

  string cad_filePath;
  Base::Placement tf_F2C;
  Base::Placement tf_F2T;

  auto result = PartUtility::importCADFile( pcDoc, cad_filePath, Robot::As_ToolOBJ);
  if (result) {
    auto toolPtr = dynamic_cast<ToolObject *>(
        pcDoc->getObject(result->getNameInDocument()));
    if (toolPtr == nullptr)
      return false;
    return toolPtr->setupToolObject(filePath, cad_filePath, tf_F2C, tf_F2T);
  } else {
    Base::Console().Error("Tool Model File import Error!\n");
  }
  return false;
}

const std::string ToolObject::getToolNameInDocument() const {
  if (getNameInDocument())
    return string(getNameInDocument());
  return string();
}

bool ToolObject::setupToolObject(const string &filePath,
                                 const string &cad_Path,
                                 const Base::Placement &tf_f2c,
                                 const Base::Placement &tf_f2t) {
  FilePath.setValue(filePath);
  CAD_File.setValue(cad_Path);
  return true;
}

bool ToolObject::loadShape(const string &filePath, const ShapeType t_Type)
{
    if(filePath.empty())
        return false;
    switch(t_Type){
    case ShapeType::STP_Shape:{
        auto result = PartUtility::importStepModel(filePath.c_str(),true);
        if(result.empty() || result.front().IsNull())
            return false;
        Shape.setValue(result.front());
        break;
    }
    case ShapeType::IGS_Shape:{
        auto result = PartUtility::importIgesModel(filePath.c_str(),true);
        if(result.empty() || result.front().IsNull())
            return false;
        Shape.setValue(result.front());
        break;
    }
    case ShapeType::MSH_Shape:{
//        MeshUtility::generateMeshNode()
        break;
    }
    }
}

const PropertyLinkSub &ToolObject::getSelectedFace() const
{
    return LinkedFaceFeature;
}

const PropertyLinkSub &ToolObject::getSelectedEdge() const
{
    return LinkedEdgeFeature;
}

const Base::Placement ToolObject::calculateSelectedFeatureCenter()
{
    return CAD_Utility::calculateLinkedFaceCenter(getSelectedFace());
}

void ToolObject::setNewMountOrigin(const Base::Placement &t_MountPose)
{
    Trans_O2M.setValue(Placement.getValue().inverse()*t_MountPose);
    Pose_Mount.setValue(t_MountPose);
}

void ToolObject::setNewTipPosition(const Base::Placement &t_TipPose)
{
    Trans_M2T.setValue(Pose_Mount.getValue().inverse()*
                       t_TipPose);
}

// To frontier(laser scanner)
const Base::Placement ToolObject::getTransform_Origin2Front() const
{
    return Trans_O2M.getValue() * getTransform_Mount2Front();
}

const Base::Placement ToolObject::getTransform_Mount2Front() const
{
    return Trans_M2T.getValue() * Trans_T2F.getValue();
}

const Base::Placement ToolObject::getPose_ABSToolTip() const
{
    return Pose_Mount.getValue() * Trans_M2T.getValue();
}

const Base::Placement ToolObject::getPose_ABSToolFront() const
{
    return Pose_Mount.getValue() * Trans_M2T.getValue() * Trans_T2F.getValue();
}

bool ToolObject::isFloating()
{
    return ParentRobotName.isEmpty();
}

bool ToolObject::assembleToRobot(const string &robotName) {
  if (robotName.empty()) {
    return false;
  }
  auto targetRobot_Ptr = dynamic_cast<Robot::Robot6AxisObject*>(getDocument()->getObject(robotName.c_str()));
  if(targetRobot_Ptr == nullptr)
      return false;
  targetRobot_Ptr->installTool(this->getNameInDocument());
  ParentRobotName.setValue(targetRobot_Ptr->getNameInDocument());
  return true;
}

void ToolObject::updateToolMountPose(const Base::Placement &t_Pose)
{
    Pose_Mount.setValue(t_Pose);
}

bool ToolObject::detachToolFromRobot() {
    auto targetRobot_Ptr = dynamic_cast<Robot::Robot6AxisObject*>(getDocument()->getObject(ParentRobotName.getValue()));
    if(targetRobot_Ptr == nullptr)
        return false;
    targetRobot_Ptr->uninstallTool(this);
    ParentRobotName.setValue("");
    return true;
}


bool ToolObject::attachObject(DocumentObject *t_obj) {
  return true;
}

bool ToolObject::detachObjcet(DocumentObject *t_obj) {
  return true;
}

void ToolObject::Save(Base::Writer &writer) const {
  Part::Feature::Save(writer);
}

void ToolObject::Restore(Base::XMLReader &reader) {
  Part::Feature::Restore(reader);
}

void ToolObject::onChanged(const Property *prop)
{
    if(prop == &Trans_O2M || prop == &Pose_Mount){
       updateToolPose();
    }
    Part::Feature::onChanged(prop);
}

void ToolObject::onDocumentRestored() {
    if(!ParentRobotName.isEmpty()){
        assembleToRobot(std::string(ParentRobotName.getValue()));
    }
    Part::Feature::onDocumentRestored();
}

void ToolObject::updateToolPose()
{
    Placement.setValue(Pose_Mount.getValue()*Trans_O2M.getValue().inverse());
}
