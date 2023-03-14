// Created by Yixiao 2022/04/24
#ifndef ROBOT_TOOLOBJECT_H
#define ROBOT_TOOLOBJECT_H
#include <App/GeoFeature.h>
#include <Eigen/Geometry>
#include <Mod/Robot/App/Utilites/CAD_Utility.h>
#include <Mod/Part/App/PartFeature.h>

#include "Mod/Robot/App/Utilites/FrameObject.h"

namespace Robot {

class Robot6AxisObject;

enum class ToolType{
    NoTool = 0,
    WeldTorch,
    Scanner,
    DepthCamera,
    RGBCamera,
    Gripper,
    Sucker
};

//enum class PosePosition { Tip, Flan };

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
  static bool loadTool(App::Document *pcDoc, std::string const &filePath);
  bool setupToolObject(const std::string &filePath,
                       const std::string &cad_Path,
                       const Base::Placement &tf_f2c,
                       const Base::Placement &tf_f2t);

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
  // Translation
  App::PropertyPlacement Pose_Mount;
  App::PropertyPlacement Trans_O2M; // Translation, Origin to Mount
  App::PropertyPlacement Trans_M2T; // Translation, Mount to Tip
  App::PropertyPlacement Trans_T2F; // Translation, Tip to Front
  // Shape
  App::PropertyString    File_Solid;
  App::PropertyString    File_Param;
  // Assemble
  App::PropertyString    MountedRobot;
  App::PropertyInteger   Type;
  // Linked Feature
  App::PropertyLinkSub   LinkedFaceFeature;
  App::PropertyLinkSub   LinkedEdgeFeature;
  App::PropertyBool      setEdit;

protected:
  ToolType m_Type = ToolType::NoTool;
//  Base::Placement        m_OriginBase;
};
}

#endif // ROBOT_TOOLOBJECT_H
