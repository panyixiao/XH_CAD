// Created by Yixiao 2022/04/24
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _ProComp_
#endif

#include "ViewProviderToolObject.h"
#include "TaskDlgToolSetupPanel.h"
#include <App/Application.h>
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <functional>

using namespace std::placeholders;

using namespace Robot;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderToolObject, PartGui::ViewProviderPartExt)

ViewProviderToolObject::ViewProviderToolObject() {}

ViewProviderToolObject::~ViewProviderToolObject() {}

void ViewProviderToolObject::updateData(const App::Property *prop) {

  Robot::ToolObject * toolPtr = static_cast<Robot::ToolObject *>(pcObject);

//  if (prop == &toolPtr->ToolFrame) {
//    claimChildren();
//  }

//  else if (prop == &toolPtr->Shape) {
//      bool shapeisNull = toolPtr->Shape.getValue().IsNull();
//      if(shapeisNull)
//        int shapeSet = 0;
//  }

////  else if (prop == &toolPtr->Pose_Tip) {
////    if (toolPtr->getAssembledRobotPtr())
////      toolPtr->updateFramePlacement();
////  }

//  else if (prop == &toolPtr->CAD_File){
//      auto filePath = toolPtr->CAD_File.getValue();
//  }
   if(prop == &toolPtr->setEdit){
        if(toolPtr->setEdit.getValue()){
            this->setEdit(Gui::ViewProvider::EditMode::Default);
        }
    }
  ViewProviderPartExt::updateData(prop);
}

void ViewProviderToolObject::attach(App::DocumentObject *obj) {
//  m_tool = dynamic_cast<Robot::ToolObject *>(obj);
  //  if (m_tool != nullptr) {
  //    m_tool->generateToolFrame();
  //  }
  ViewProviderPartExt::attach(obj);
}

bool ViewProviderToolObject::setEdit(int ModNum) {
  if (ModNum == Gui::ViewProvider::EditMode::Default) {
    auto t_ToolPtr = static_cast<Robot::ToolObject *>(pcObject);
    Gui::TaskView::TaskDialog *dlg = new TaskDlgToolObject(t_ToolPtr);
    if(dlg == nullptr)
        return false;
    Gui::Control().showDialog(dlg);
  }
  return true;
}

void ViewProviderToolObject::setDisplayMode(const char *displayModeName) {
  ViewProviderPartExt::setDisplayMode(displayModeName);
}

std::vector<std::string> ViewProviderToolObject::getDisplayModes() const {
  return ViewProviderPartExt::getDisplayModes();
}

bool ViewProviderToolObject::doubleClicked() {
  return this->setEdit(Gui::ViewProvider::EditMode::Default);
}

bool ViewProviderToolObject::onDelete(const std::vector<std::string> &subNames) {
  auto toolPtr = static_cast<Robot::ToolObject*>(pcObject);
  if (toolPtr == nullptr)
    return false;
  //  Gui::Document
  if (std::strcmp(App::GetApplication().getActiveDocument()->getName(),
                  toolPtr->getDocument()->getName())) {
    std::string msg;
    msg = std::string(toolPtr->getNameInDocument()) +
            std::string(" doesn't belong to current active document, can't be deleted!\n");
    Base::Console().Message(msg.c_str());
    return false;
  }
  // Clear Frame Object
//  if (toolPtr->getToolFramePtr()) {
//    toolPtr->getDocument()->removeObject(
//        toolPtr->getToolFramePtr()->getNameInDocument());
//  }
  // Clear Mesh

  // Clear Robot EndEffector
  toolPtr->detachToolFromRobot();
//  toolPtr->unregisterTool_inLib();
  return true;
}

std::vector<App::DocumentObject *> ViewProviderToolObject::claimChildren() const {
  std::vector<App::DocumentObject *> children;
//  auto toolPtr = static_cast<Robot::ToolObject*>(pcObject);
//  if (toolPtr && toolPtr->ToolFrame.getStrValue().size()) {
//    auto ToolFrameObj = toolPtr->getDocument()->getObject(
//        toolPtr->ToolFrame.getStrValue().c_str());
//    if (ToolFrameObj)
//      children.push_back(ToolFrameObj);
//  }
  return children;
}
