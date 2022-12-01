// Created by Yixiao 2017/01/19
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "DraggerUtility.h"
#include <Gui/Document.h>
#include <Gui/SoFCUnifiedSelection.h>
#include <Gui/View3DInventor.h>

#include <Inventor/draggers/SoCenterballDragger.h>
#include <Inventor/draggers/SoTrackballDragger.h>
#include <Inventor/draggers/SoTransformBoxDragger.h>
#include <Inventor/nodes/SoPickStyle.h>

#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Mod/Robot/App/Utilites/DS_Utility.h>

using namespace RobotGui;
//using namespace RD_CAD_Utility;

InteractiveDragger::InteractiveDragger() {
}

InteractiveDragger::InteractiveDragger(SoSeparator *root,
                                         const Base::Placement &initPose,
                                         DraggerUsage usage, DraggerType d_Type) {
  if (!createDragger(root, initPose, usage, d_Type))
    return;
}

InteractiveDragger::~InteractiveDragger() {}

bool InteractiveDragger::createDragger(SoSeparator *root,
                                       const Base::Placement &initPose,
                                       DraggerUsage usage,
                                       DraggerType d_Type) {
    if (root == nullptr)
        return false;
    m_root = root;
    m_currPose = initPose;
    SoTransform *initTrans = new SoTransform;
    initTrans->ref();
    Robot::DS_Utility::convert_Placement2Transform(initPose, initTrans);

    m_Type = d_Type;
//    if(usage == DraggerUsage::Display){
//        m_dragger = new SoFCCSysDragger(true);
//        auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
//        dragger->draggerSize.setValue(0.02);
//    }
//    else{
//        switch (m_Type) {
//        case DraggerType::FCCsysDragger:{
//            m_dragger = new SoFCCSysDragger(true);
//            auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
//            dragger->draggerSize.setValue(0.02);
//            dragger->translationIncrement.setValue(1.0);
//            dragger->rotationIncrement.setValue(1 * M_PI / 180.0);
//        }
//            break;
//        case DraggerType::SpotLightDragger:
//            m_dragger = new SoSpotLightDragger();
//            break;
//        case DraggerType::CenterballDragger:
//            m_dragger = new SoCenterballDragger();
//            break;
//        default:
//            break;
//        }
//    }
    switch (m_Type) {
    case DraggerType::FCCsysDragger:{
        m_dragger = new SoFCCSysDragger();
        auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
        dragger->draggerSize.setValue(0.02);
        if(usage == DraggerUsage::Interaction){
            dragger->translationIncrement.setValue(1.0);
            dragger->rotationIncrement.setValue(1 * M_PI / 180.0);
        }
    }
        break;
    case DraggerType::SpotLightDragger:
        m_dragger = new SoSpotLightDragger();
        break;
    case DraggerType::CenterballDragger:
        m_dragger = new SoCenterballDragger();
        break;
    default:
        break;
    }
    m_dragger->ref();
    this->setDraggerPosition(initPose);
    m_dragger->addMotionCallback(dragMotionCallback, this);
    m_dragger->addFinishCallback(dragFinishCallback, this);
    initTrans->unref();
    m_root->insertChild(m_dragger, 0);
    return true;
}

void InteractiveDragger::destroyDragger() {
    if (m_dragger != nullptr) {
      m_root->removeChild(m_dragger);
      m_dragger->unref();
      m_dragger = nullptr;
    }
    enableSceneGraphSelection(true);
}

bool InteractiveDragger::setAttachingViewProvider(ViewProvider *p) {
  if (p == nullptr)
    return false;
  Gui::Document *doc = Gui::Application::Instance->activeDocument();
  auto view = dynamic_cast<View3DInventor *>(doc->getViewOfViewProvider(p));
  if (view == nullptr){
      return false;
  }
  m_3dInventorViewer = view->getViewer();
  auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
  if (dragger!=nullptr && m_3dInventorViewer) {
    dragger->setUpAutoScale(m_3dInventorViewer->getSoRenderManager()->getCamera());
    return true;
  }
  return true;
}

bool InteractiveDragger::setDraggerPosition(const Base::Placement &newPose) {
    SoTransform *tmp_Trans = new SoTransform();
    tmp_Trans->ref();
    if (m_dragger == nullptr)
      return false;
    m_lastPose = newPose;
    auto tmp_Pose = Base::Placement(newPose);

    Robot::DS_Utility::convert_Placement2Transform(tmp_Pose, tmp_Trans);
    switch(m_Type){
        case DraggerType::FCCsysDragger:{
            auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
            dragger->translation.setValue(tmp_Trans->translation.getValue());
            dragger->rotation.setValue(tmp_Trans->rotation.getValue());
        }
            break;
        case DraggerType::CenterballDragger:{
            auto dragger = dynamic_cast<SoCenterballDragger*>(m_dragger);
            dragger->rotation.setValue(tmp_Trans->rotation.getValue());
        }
            break;
        case DraggerType::SpotLightDragger:{
            auto dragger = dynamic_cast<SoSpotLightDragger*>(m_dragger);
            dragger->translation.setValue(tmp_Trans->translation.getValue());
            dragger->rotation.setValue(tmp_Trans->rotation.getValue());
        }
        break;
    }
    tmp_Trans->unref();
    return true;
}

void InteractiveDragger::setUpAutoScale() {
    if (m_dragger != nullptr)
//        m_dragger->setUpAutoScale(cameraIn);
        m_dragger->setMinScale(2);
}

void InteractiveDragger::setConnectedTransformNode(SoTransform *target_Node){
  if (target_Node == nullptr)
    return;
  m_ConnectedNode = target_Node;

  switch(m_Type){
      case DraggerType::FCCsysDragger:{
          auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
          m_ConnectedNode->translation.connectFrom(&dragger->translation);
          m_ConnectedNode->rotation.connectFrom(&dragger->rotation);
      }
          break;
      case DraggerType::CenterballDragger:{
          auto dragger = dynamic_cast<SoCenterballDragger*>(m_dragger);
          m_ConnectedNode->rotation.connectFrom(&dragger->rotation);
      }
          break;
      case DraggerType::SpotLightDragger:{
          auto dragger = dynamic_cast<SoSpotLightDragger*>(m_dragger);
          m_ConnectedNode->translation.connectFrom(&dragger->translation);
          m_ConnectedNode->rotation.connectFrom(&dragger->rotation);
      }
      break;
      default:
          break;
  }
}

void InteractiveDragger::enableSceneGraphSelection(bool enable) {
  if (m_3dInventorViewer == nullptr)
    return;
  if (enable) {
    SoNode *child =
        static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph())
            ->getChild(0);
    if (child && child->isOfType(SoPickStyle::getClassTypeId()))
      static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph())
          ->removeChild(child);

  } else {
    SoPickStyle *rootPickStyle = new SoPickStyle();
    rootPickStyle->style = SoPickStyle::UNPICKABLE;
    static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph())
        ->insertChild(rootPickStyle, 0);
  }
}

void InteractiveDragger::setDraggerMovable(bool movable) {
  Dragger_Movable = movable;
}

 void InteractiveDragger::disConnectNode() {
  if (m_ConnectedNode == nullptr)
    return;

  switch(m_Type){
      case DraggerType::FCCsysDragger:{
          auto dragger = dynamic_cast<SoFCCSysDragger*>(m_dragger);
          m_ConnectedNode->translation.disconnect(&dragger->translation);
          m_ConnectedNode->rotation.disconnect(&dragger->rotation);
      }
          break;
      case DraggerType::CenterballDragger:{
          auto dragger = dynamic_cast<SoCenterballDragger*>(m_dragger);
          m_ConnectedNode->rotation.disconnect(&dragger->rotation);
      }
          break;
      case DraggerType::SpotLightDragger:{
          auto dragger = dynamic_cast<SoSpotLightDragger*>(m_dragger);
          m_ConnectedNode->translation.disconnect(&dragger->translation);
          m_ConnectedNode->rotation.disconnect(&dragger->rotation);
      }
      break;
      default:
          break;
  }
  m_ConnectedNode = nullptr;
 }

 const DraggerType &InteractiveDragger::getDraggerType() const
 {
     return m_Type;
 }

void InteractiveDragger::dragStartCallback(void *data, SoDragger *d) {
  Gui::Application::Instance->activeDocument()->openCommand("Transform");
}

void InteractiveDragger::dragMotionCallback(void *data, SoDragger *d) {

  Gui::Application::Instance->activeDocument()->openCommand("Transform");
  auto util_Dragger = reinterpret_cast<InteractiveDragger *>(data);
  SoDragger* t_dragger;
  switch(util_Dragger->getDraggerType()){
  case DraggerType::FCCsysDragger:
      t_dragger = dynamic_cast<SoFCCSysDragger *>(d);
      break;
  case DraggerType::SpotLightDragger:
      t_dragger = dynamic_cast<SoSpotLightDragger *>(d);
      break;
  case DraggerType::CenterballDragger:
      t_dragger = dynamic_cast<SoCenterballDragger *>(d);
      break;
  }
  if (util_Dragger->Dragger_Movable) {
    util_Dragger->draggerMoving_flag = true;
    updateDraggerPose(util_Dragger->m_currPose, t_dragger);

    if(util_Dragger->m_poseCallBack)
      util_Dragger->m_poseCallBack(util_Dragger->m_currPose);

    if(util_Dragger->m_incCallBack)
        util_Dragger->m_incCallBack(util_Dragger);
    util_Dragger->draggerMoving_flag = false;
  }
  util_Dragger->setDraggerPosition(util_Dragger->m_currPose);
  Gui::Application::Instance->activeDocument()->commitCommand();
}

void InteractiveDragger::dragFinishCallback(void *data, SoDragger *d)
{
    Gui::Application::Instance->activeDocument()->openCommand("Transform");
    auto util_Dragger = reinterpret_cast<InteractiveDragger *>(data);
    SoDragger* t_dragger;
    switch(util_Dragger->getDraggerType()){
    case DraggerType::FCCsysDragger:
        t_dragger = dynamic_cast<SoFCCSysDragger *>(d);
        break;
    case DraggerType::SpotLightDragger:
        t_dragger = dynamic_cast<SoSpotLightDragger *>(d);
        break;
    case DraggerType::CenterballDragger:
        t_dragger = dynamic_cast<SoCenterballDragger *>(d);
        break;
    }
    if(util_Dragger->m_finishCallBack){
        util_Dragger->m_finishCallBack(util_Dragger);
    }
    Gui::Application::Instance->activeDocument()->commitCommand();
}

void InteractiveDragger::updateDraggerPose(
    Base::Placement &poseToBeUpdate, SoDragger *draggerIn) {
  if (draggerIn == nullptr)
    return;

  SbMatrix currentPoseMat = draggerIn->getMotionMatrix();
  SbVec3f transform;
  SbRotation rotation;
  SbVec3f scale;
  SbRotation soRot;
  currentPoseMat.getTransform(transform,
                              rotation,
                              scale,
                              soRot);

  poseToBeUpdate.setPosition(Base::Vector3d(transform.getValue()[0],
                                         transform.getValue()[1],
                                         transform.getValue()[2]));

  poseToBeUpdate.setRotation(Base::Rotation(rotation.getValue()[0], rotation.getValue()[1],
                                         rotation.getValue()[2], rotation.getValue()[3]));
  //    dragger_TypeMatch->getMotionMatrix()
}
