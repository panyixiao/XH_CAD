// Created by Yixiao 2023/06/06

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBox_FeaturePathPanel.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Document.h>
#include "Mod/Robot/Gui/ui_TaskBox_FeaturePathPanel.h"

using namespace RobotGui;

TaskBox_FeaturePathPanel::TaskBox_FeaturePathPanel(Robot::TaskObject *t_TaskPtr,
                                                   QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Task Manager"), true, parent) {
  if (t_TaskPtr == nullptr)
    return;

  m_currentDoc = t_TaskPtr->getDocument();
  m_targetTaskPtr = t_TaskPtr;
  initUi();
}

bool TaskBox_FeaturePathPanel::accept() {

}

bool TaskBox_FeaturePathPanel::reject() {

}


void TaskBox_FeaturePathPanel::initUi() {
    m_proxy = new QWidget();
    m_ui_New = new Ui_TaskBox_FeaturePathPanel;
    m_ui_New->setupUi(m_proxy);
    this->groupLayout()->addWidget(m_proxy, Qt::AlignTop);
}

#include "moc_TaskBox_FeaturePathPanel.cpp"
