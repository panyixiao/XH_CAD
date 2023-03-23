// Created by Yixiao 20220424
#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "ToolObject.h"
#include "Mod/Robot/App/Utilites/MeshUtility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/App/Utilites/PartUtility.h"
//#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"
#include "Mod/Robot/App/Mechanics/MechanicRobot.h"

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
  ADD_PROPERTY_TYPE(ToolBrand, (""),"Property", Prop_None,"Name of Tool Brand");
  ADD_PROPERTY_TYPE(FilePath_Solid, (""), "File", Prop_None, "Path to Model File");
  ADD_PROPERTY_TYPE(FilePath_Param, (""), "File", Prop_None, "Path to Tool File");
  ADD_PROPERTY_TYPE(MountedRobot, (""), "Assemble", Prop_None, "Assembled Robot Name");
  ADD_PROPERTY_TYPE(_ToolType, (0), "Property", Prop_None, "Tool Type");

  ADD_PROPERTY(Pose_Mount, (Base::Placement()));
  ADD_PROPERTY(Trans_O2M, (Base::Placement()));
  ADD_PROPERTY(Trans_M2T, (Base::Placement()));
  ADD_PROPERTY(Trans_T2F, (Base::Placement()));
  ADD_PROPERTY(setEdit, (false));
  FilePath_Solid.setStatus(App::Property::Status::ReadOnly, true);
  FilePath_Param.setStatus(App::Property::Status::ReadOnly, true);
  MountedRobot.setStatus(App::Property::Status::ReadOnly, true);
  Placement.setStatus(App::Property::Status::Hidden, true);

  setEdit.setStatus(App::Property::Status::ReadOnly, true);

  ADD_PROPERTY_TYPE(LinkedEdgeFeature, (0), "", Prop_None, "Linked Edge Feature for attachment and tip setup");
  LinkedEdgeFeature.setStatus(App::Property::Status::Hidden, true);
  ADD_PROPERTY_TYPE(LinkedFaceFeature, (0), "", Prop_None, "Linked Face Feature for attachment and tip setup");
  LinkedFaceFeature.setStatus(App::Property::Status::Hidden, true);
}

ToolObject::~ToolObject() {}

bool ToolObject::loadTool(const string &param_FilePath) {
  if (param_FilePath.empty())
      return false;
  if(!m_FileOperator.openFile(param_FilePath, QIODevice::ReadOnly))
      return false;
  Trans_O2M.setValue(m_FileOperator.readPosePropFromFile(QObject::tr("Trans_O2M")));
  Trans_M2T.setValue(m_FileOperator.readPosePropFromFile(QObject::tr("Trans_M2T")));
  Trans_T2F.setValue(m_FileOperator.readPosePropFromFile(QObject::tr("Trans_T2F")));
  FilePath_Param.setValue(m_FileOperator.readStringPropFromFile(QObject::tr("FilePath_Param")));
  FilePath_Solid.setValue(m_FileOperator.readStringPropFromFile(QObject::tr("FilePath_Solid")));
  ToolBrand.setValue(m_FileOperator.readStringPropFromFile(QObject::tr("ToolBrand")));
  _ToolType.setValue(m_FileOperator.readNumberPropFromFile(QObject::tr("CurrentType")));
  m_FileOperator.closeFile();

  bool result = false;
  auto modelFilePath = FilePath_Solid.getStrValue();
  Base::FileInfo file(modelFilePath.c_str());
  if(file.hasExtension("stp") || file.hasExtension("step")){
      result = loadShape(modelFilePath, Robot::ShapeType::STP_Shape);
  }
  else if(file.hasExtension("igs") || file.hasExtension("iges")){
      result = loadShape(modelFilePath, Robot::ShapeType::IGS_Shape);
  }
  return result;
}

const std::string ToolObject::getToolNameInDocument() const {
  if (getNameInDocument())
    return string(getNameInDocument());
  return string();
}

bool ToolObject::saveTool()
{
    m_FileOperator.openFile(FilePath_Param.getStrValue());
    m_FileOperator.insertItem(QObject::tr("Trans_O2M"),FileOperator::poseToJsonObject(Trans_O2M.getValue()));
    m_FileOperator.insertItem(QObject::tr("Trans_M2T"),FileOperator::poseToJsonObject(Trans_M2T.getValue()));
    m_FileOperator.insertItem(QObject::tr("Trans_T2F"),FileOperator::poseToJsonObject(Trans_T2F.getValue()));
    m_FileOperator.insertItem(QObject::tr("FilePath_Solid"),QString::fromLocal8Bit(FilePath_Solid.getValue()));
    m_FileOperator.insertItem(QObject::tr("FilePath_Param"),QString::fromLocal8Bit(FilePath_Param.getValue()));
    m_FileOperator.insertItem(QObject::tr("ToolBrand"),QString::fromLocal8Bit(ToolBrand.getValue()));
    m_FileOperator.insertItem(QObject::tr("CurrentType"),QString::number(_ToolType.getValue()));
    return m_FileOperator.saveFile();
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

const Base::Placement ToolObject::getPose_ABSToolMount() const
{
    return Placement.getValue() * Trans_O2M.getValue();
}

const Base::Placement ToolObject::getPose_ABSToolTip() const
{
    return getPose_ABSToolMount() * Trans_M2T.getValue();
}

const Base::Placement ToolObject::getPose_ABSToolFront() const
{
    return getPose_ABSToolTip() * Trans_T2F.getValue();
}

bool ToolObject::isFloating()
{
    return MountedRobot.isEmpty();
}

bool ToolObject::assembleToRobot(const string &robotName) {
  if (robotName.empty()) {
    return false;
  }
  auto targetRobot_Ptr = dynamic_cast<Robot::MechanicRobot*>(getDocument()->getObject(robotName.c_str()));
  if(targetRobot_Ptr == nullptr)
      return false;
  if(targetRobot_Ptr->installTool(this->getNameInDocument())){
      MountedRobot.setValue(targetRobot_Ptr->getNameInDocument());
      return true;
  }
  return false;
}

void ToolObject::updateToolMountPose(const Base::Placement &t_Pose)
{
    Pose_Mount.setValue(t_Pose);
}

bool ToolObject::detachToolFromRobot() {
    auto targetRobot_Ptr = dynamic_cast<Robot::MechanicRobot*>(getDocument()->getObject(MountedRobot.getValue()));
    if(targetRobot_Ptr == nullptr)
        return false;
    targetRobot_Ptr->uninstallTool(this->getNameInDocument());
    MountedRobot.setValue("");
    auto newPose = targetRobot_Ptr->Placement.getValue().getPosition();
    newPose.x += 500;
    newPose.y += 500;
    Pose_Mount.setValue(Base::Placement(newPose, Base::Rotation()));
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
//    else if(prop == &_ToolType){
//        m_Type = (ToolType)_ToolType.getValue();
//    }
    Part::Feature::onChanged(prop);
}

void ToolObject::onDocumentRestored() {
    if(!MountedRobot.isEmpty()){
        assembleToRobot(std::string(MountedRobot.getValue()));
    }
    Part::Feature::onDocumentRestored();
}

void ToolObject::updateToolPose()
{
    Placement.setValue(Pose_Mount.getValue()*Trans_O2M.getValue().inverse());
}
