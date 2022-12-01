#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <App/Document.h>
#include <Gui/Command.h>
#include <Gui/SoFCSelection.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoMarkerSet.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

#include "ViewProviderFrameObject.h"

//#include <Mod/RD_Setup/App/rd_PlanningObject.h>
//#include <Mod/RD_Setup/App/rd_ToolObject.h>

PROPERTY_SOURCE(RobotGui::ViewProviderFrameObject,
                Gui::ViewProviderGeometryObject)

//using namespace RD_SetupGui;
using namespace RobotGui;

ViewProviderFrameObject::ViewProviderFrameObject() {
  m_frameOriginRoot = new Gui::SoFCSelection();
  m_frameOriginRoot->highlightMode = Gui::SoFCSelection::ON;
  m_frameOriginRoot->ref();
  //  m_OperationDragger = make_unique<Gui_Utility_Dragger*>(new
  //  Gui_Utility_Dragger())
}
ViewProviderFrameObject::~ViewProviderFrameObject() { m_frameOriginRoot->unref(); }

void ViewProviderFrameObject::attach(App::DocumentObject *pcObject) {
  if (pcObject == nullptr)
    return;
  auto target_Frame = static_cast<Robot::FrameObject *>(pcObject);
  addDisplayMaskMode(m_frameOriginRoot, "Visible");
  m_frameOriginRoot->objectName = target_Frame->getNameInDocument();
  m_frameOriginRoot->documentName = target_Frame->getDocument()->getName();
  m_Frame = target_Frame;

  ViewProviderGeometryObject::attach(pcObject);
}

void ViewProviderFrameObject::setDisplayMode(const char *ModeName) {
  if (strcmp("Visible", ModeName) == 0) {
    setDisplayMaskMode("Visible");
  }
  ViewProviderGeometryObject::setDisplayMode(ModeName);
}

std::vector<std::string> ViewProviderFrameObject::getDisplayModes() const {
  std::vector<std::string> StrList;
  StrList.push_back("Visible");
  return StrList;
}

void ViewProviderFrameObject::updateData(const App::Property *prop) {
    if (prop == &m_Frame->ChildrenList) {
        claimChildren();
    }
    else if (prop == &m_Frame->ParentFrameID) {
          m_Frame->insertToTargetFrame(m_Frame->ParentFrameID.getStrValue());
    }
  ViewProviderGeometryObject::updateData(prop);
}

void ViewProviderFrameObject::onChanged(const App::Property *prop) {
  ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderFrameObject::setEdit(int ModNum) {
  //  ViewProviderGeometryObject::setEdit(ModNum);
  setupDragger(true);
  return true;
}

void ViewProviderFrameObject::unsetEdit(int ModNum) {
  //  ViewProviderGeometryObject::unsetEdit(ModNum);
  setupDragger(false);
}

void ViewProviderFrameObject::dragObject(App::DocumentObject *t_obj) {
//  m_Frame->removeObjectFromFrame(t_obj);
}

void ViewProviderFrameObject::dropObject(App::DocumentObject *t_obj) {
//  m_Frame->addObjectIntoFrame(t_obj);
}

bool ViewProviderFrameObject::onDelete(const std::vector<std::string> &subNames) {
  // Clear Children
  m_Frame->resetChildrenParentFrame();
  // Clear Parent
  if (!m_Frame->ParentFrameID.getStrValue().empty()) {
    auto parentFramePtr =
        dynamic_cast<Robot::FrameObject *>(m_Frame->getDocument()->getObject(
            m_Frame->ParentFrameID.getStrValue().c_str()));
    if (parentFramePtr)
      parentFramePtr->removeObjectFromFrame(m_Frame);
  }
  // Clear Attaching Object
  if (!m_Frame->Owner.getStrValue().empty()) {
    auto ownPtr =
        m_Frame->getDocument()->getObject(m_Frame->Owner.getStrValue().c_str());
    if (ownPtr != nullptr) {
//      if (ownPtr->isDerivedFrom(
//              Robot::rd_PlanningObject::getClassTypeId())) {
//        auto planningObj = dynamic_cast<RD_Setup::rd_PlanningObject *>(ownPtr);
//        planningObj->CurrentFrame.setValue(m_Frame->getWorldPlanningFrame());
//      } else if (ownPtr->isDerivedFrom(
//                     RD_Setup::RD_ToolObject::getClassTypeId())) {
//        auto toolObj = dynamic_cast<RD_Setup::RD_ToolObject *>(ownPtr);
//        toolObj->ToolFrame.setValue(m_Frame->getWorldPlanningFrame());
//      }
    }
  }
  return true;
}

std::vector<App::DocumentObject *> ViewProviderFrameObject::claimChildren() const {
  return m_Frame->getObjectsInFrame();
}

void ViewProviderFrameObject::setupDragger(bool enable) {
  int i = 0;
  if (enable) {
    if (m_OperationDragger == nullptr) {
      m_OperationDragger = std::make_unique<RobotGui::InteractiveDragger>();
      m_OperationDragger->createDragger(getRoot(),
                                        m_Frame->getPlacement(),
                                        DraggerUsage::Interaction);
      m_OperationDragger->setAttachingViewProvider(this);
      m_OperationDragger->setup_poseCallback(
          std::bind(&ViewProviderFrameObject::updatePlacement, this,
                    std::placeholders::_1));
    }

  } else {
    if (m_OperationDragger != nullptr)
      m_OperationDragger->destroyDragger();
    m_OperationDragger = nullptr;
  }
}

void ViewProviderFrameObject::updatePlacement(
    const Base::Placement &newPlacement) {
  m_Frame->updateFramePlacement(newPlacement);
}
