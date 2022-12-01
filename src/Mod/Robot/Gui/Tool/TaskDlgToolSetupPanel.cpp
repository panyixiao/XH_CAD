// Created by Yixiao 2022/04/24

#ifndef _PreComp_
#endif
#include "Mod/Robot/Gui/PreCompiled.h"
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <functional>

#include "ViewProviderToolObject.h"

#include "TaskDlgToolSetupPanel.h"
#include "ViewProviderToolObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"

using namespace std::placeholders;
using namespace RobotGui;

TaskDlgToolObject::TaskDlgToolObject(Robot::ToolObject *targetTool, QWidget *parent): TaskDialog()
{
    if(targetTool == nullptr)
        return;

    m_toolObj_VP = dynamic_cast<ViewProviderToolObject*>(Gui::Application::Instance->activeDocument()->getViewProvider(targetTool));

    int maxium_width = 350;
    // Face Selection
    auto m_FaceSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Face COUNT 1",
                                                                &(targetTool->LinkedFaceFeature));
    m_FaceSelection->hide();
    Content.push_back(m_FaceSelection);
    // Edge Selection
    auto m_EdgeSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Edge COUNT 1",
                                                                &(targetTool->LinkedEdgeFeature));
    m_EdgeSelection->hide();
    Content.push_back(m_EdgeSelection);

    switch(targetTool->getToolType()){
    case ToolType::WeldTorch:{
        auto m_TorchSetupPanel = new TaskBoxTorchToolSetupPanel(targetTool,
                                                                m_FaceSelection,
                                                                m_EdgeSelection);
        QObject::connect(m_TorchSetupPanel, SIGNAL(Signal_updateDraggerPose(Base::Placement)),
                         this, SLOT(slot_updateDraggerPose(Base::Placement)));
        Content.push_back(m_TorchSetupPanel);
    }
        break;
    case ToolType::Scanner:{
        auto m_ScannerSetupPanel = new TaskBoxLaserScannerSetupPanel(targetTool,
                                                                m_FaceSelection,
                                                                m_EdgeSelection);
        QObject::connect(m_ScannerSetupPanel, SIGNAL(Signal_updateDraggerPose(Base::Placement)),
                         this, SLOT(slot_updateDraggerPose(Base::Placement)));
        Content.push_back(m_ScannerSetupPanel);
    }
        break;
    }

    setButtonPosition(ButtonPosition::South);
    setDlgWdith(maxium_width+5);

    createDragger(targetTool->Pose_Mount.getValue());
}

bool TaskDlgToolObject::accept() {
    destroyDragger();
  return true;
}

bool TaskDlgToolObject::reject() {
    destroyDragger();
  return true;
}

bool TaskDlgToolObject::createDragger(const Base::Placement &init_Pose) {
  if (m_toolObj_VP == nullptr || m_displayDragger!=nullptr)
    return false;
  m_displayDragger = std::make_shared<InteractiveDragger>();
  m_displayDragger->createDragger(m_toolObj_VP->getRoot(),
                                  init_Pose,
                                  DraggerUsage::Display);
  m_displayDragger->setAttachingViewProvider(m_toolObj_VP);

  return true;
}


void TaskDlgToolObject::destroyDragger() {
  if (m_displayDragger == nullptr)
    return;
  m_displayDragger->destroyDragger();
}

void TaskDlgToolObject::slot_updateDraggerPose(const Base::Placement newPlacement) {
  if (m_displayDragger != nullptr)
    m_displayDragger->setDraggerPosition(newPlacement);
}

#include "moc_TaskDlgToolSetupPanel.cpp"
