// Created by Yixiao 2017/01/19
#ifndef DRAGGER_UTILITY_H
#define DRAGGER_UTILITY_H

#include <Mod/Robot/Gui/PreCompiled.h>
#include <Base/Placement.h>
#include <Gui/SoFCCSysDragger.h>
#include <Gui/ViewProvider.h>
#include <Inventor/draggers/SoCenterballDragger.h>
#include <Inventor/draggers/SoSpotLightDragger.h>
#include <functional>

using namespace Gui;
class SoTransform;
namespace RobotGui {

class InteractiveDragger;
typedef std::function<void(const Base::Placement &)> pose_CallBack;
typedef std::function<void(InteractiveDragger *dragger)> self_CallBack;
enum DraggerUsage { Interaction, Display };

enum class DraggerType{
    FCCsysDragger,
    CenterballDragger,
    SpotLightDragger
};

class InteractiveDragger {

public:
  InteractiveDragger();
  InteractiveDragger(SoSeparator *root,
                     const Base::Placement &initPose = Base::Placement(),
                     DraggerUsage usage = Display,
                     DraggerType d_Type = DraggerType::FCCsysDragger);
  ~InteractiveDragger();
  bool createDragger(SoSeparator *root,
                     const Base::Placement &initPose = Base::Placement(),
                     DraggerUsage usage = Display,
                     DraggerType d_Type = DraggerType::FCCsysDragger);
  void destroyDragger();
  bool setAttachingViewProvider(Gui::ViewProvider *p);
  bool setDraggerPosition(const Base::Placement &newPose);

  void setup_poseCallback(pose_CallBack f) { m_poseCallBack = f; }
  void setup_incCallback(self_CallBack f) { m_incCallBack = f; }
  void setup_finishCallback(self_CallBack f){m_finishCallBack = f;}

  bool draggerIsMoving() const { return draggerMoving_flag; }
  void setUpAutoScale();
  void setConnectedTransformNode(SoTransform *target_Node);
  void enableSceneGraphSelection(bool enable);
  void setDraggerMovable(bool movable);
  void disConnectNode();
  const DraggerType& getDraggerType() const;
  const SoDragger* getDraggerPtr() const{
      return m_dragger;
  }
  const Base::Placement& getCurrentPose() const {
      return m_currPose;
  }

  const Base::Placement& getLastPose() const {
      return m_lastPose;
  }

  void setDraggerTrans(const Base::Vector3d& new_Trans){
      m_RealTrans = new_Trans;
  }

private:
  static void dragStartCallback(void *data, SoDragger *d);
  static void dragMotionCallback(void *data, SoDragger *d);
  static void dragFinishCallback(void *data, SoDragger *d);
  static void updateDraggerPose(Base::Placement &poseToBeUpdate,
                                SoDragger *draggerIn);

private:
  SoSeparator *m_root;
  View3DInventorViewer *m_3dInventorViewer = nullptr;
  DraggerType m_Type;
  SoDragger *m_dragger = nullptr;
  float m_draggerSize = 10.0;

  SoTransform *m_ConnectedNode = nullptr;
  bool Dragger_Movable = true;
  bool draggerMoving_flag = false;
  pose_CallBack m_poseCallBack;
  self_CallBack m_incCallBack;
  self_CallBack m_finishCallBack;
  bool for_Visual_Only = false;

  Base::Placement m_currPose;
  Base::Placement m_lastPose;
  Base::Vector3d m_RealTrans;
};
}

#endif
