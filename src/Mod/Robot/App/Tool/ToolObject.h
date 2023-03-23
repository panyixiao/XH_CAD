// Created by Yixiao 2022/04/24
#ifndef ROBOT_TOOLOBJECT_H
#define ROBOT_TOOLOBJECT_H

#include "Mod/Robot/App/PreCompiled.h"
#include <App/GeoFeature.h>
#include <Eigen/Geometry>
#include <Mod/Part/App/PartFeature.h>
#include <Mod/Robot/App/Database/ToolDatabase.h>
#include <Mod/Robot/App/Database/FileOperator.h>
#include <Mod/Robot/App/Utilites/CAD_Utility.h>
#include "Mod/Robot/App/Utilites/FrameObject.h"

namespace Robot {

//class Robot6AxisObject;

enum class ToolType{
    Undefined = 0,
    WeldTorch,
    _2DScanner,
    _3DCamera,
    RGBCamera,
    Gripper,
    Sucker
};

class ToolObject : public Part::Feature {
  PROPERTY_HEADER(Robot::ToolObject);

public:
  ToolObject();
  virtual ~ToolObject();
  const char *getViewProviderName(void) const {
    return "RobotGui::ViewProviderToolObject";
  }
  virtual const std::string getToolNameInDocument() const;
  // TOOL Setup
  bool loadTool(const std::string& param_FilePath);
  bool saveTool();
//  static Tool_BasicParam parsingToolDescriptionFile(const std::string& t_filepath);

//  bool setupToolObject(const std::string &filePath,
//                       const std::string &cad_Path,
//                       const Base::Placement &tf_f2c,
//                       const Base::Placement &tf_f2t);

  bool loadStepShape(const std::string& filePath);
  bool loadIgesShape(const std::string& filePath);
  bool loadShape(const std::string& filePath, const Robot::ShapeType t_Type);
  void calculateMatrices();

  // Assemble
  const App::PropertyLinkSub& getSelectedFace() const;
  const App::PropertyLinkSub& getSelectedEdge() const;  
  const Base::Placement calculateSelectedFeatureCenter();

  void  setNewMountOrigin(const Base::Placement& t_MountPose);
  void  setNewTipPosition(const Base::Placement& t_TipPose);

  // Placement
  const Base::Placement getTransform_Origin2Front() const;
  const Base::Placement getTransform_Mount2Front() const;
  const Base::Placement getPose_ABSToolMount() const;
  const Base::Placement getPose_ABSToolTip() const;
  const Base::Placement getPose_ABSToolFront() const;

  bool isFloating();
  bool assembleToRobot(const std::string &robotName);
  bool detachToolFromRobot();
  void updateToolMountPose(const Base::Placement& t_Pose);
  void updateToolPose();

  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);

  const ToolType& getToolType() const{
      return m_Type;
  }

protected:
  /// get called by the container when a property has changed
  virtual void onChanged (const App::Property* prop);
  void onDocumentRestored() override;

public:
//  App::PropertyLink
  App::PropertyString    ToolBrand;
  // Translation
  App::PropertyPlacement Pose_Mount;
  App::PropertyPlacement Trans_O2M; // Translation, Origin to Mount
  App::PropertyPlacement Trans_M2T; // Translation, Mount to Tip
  App::PropertyPlacement Trans_T2F; // Translation, Tip to Front
  // Shape
  App::PropertyString    FilePath_Solid;
  App::PropertyString    FilePath_Param;
  // Assemble
  App::PropertyString    MountedRobot;
  App::PropertyInteger   CurToolType;
  // Linked Feature
  App::PropertyLinkSub   LinkedFaceFeature;
  App::PropertyLinkSub   LinkedEdgeFeature;
  App::PropertyBool      setEdit;

protected:
  ToolType m_Type = ToolType::Undefined;
  FileOperator m_FileOperator;
};
}

#endif
