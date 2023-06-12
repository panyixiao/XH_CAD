// Created By Yixiao 2022-05-10
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>
//#include <Mod/RD_Setup/Gui/Gui_Utility_Doc.h>
#include <Mod/Robot/Gui/Utilites/DocUtility.h>
#include <Mod/Robot/Gui/ui_TaskBox_SimulationManager.h>
//#include <g3log/rlog.h>

#include "TaskBox_SimulationManager.h"
using namespace RobotGui;
using namespace std;

TaskBox_SimulationManager::TaskBox_SimulationManager(Robot::TaskObject *t_Object,
                                                     QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Simulation"),
              true, parent) {
    if(t_Object == nullptr)
        return;
  initUi();
  m_TaskObjectPtr = t_Object;
  m_currentDoc = t_Object->getDocument();
//  m_SimulationManager = make_shared<RD_TaskManager::RD_SimulationManager>();
//  disableSimulationPanel();
}

TaskBox_SimulationManager::TaskBox_SimulationManager(Robot::RobotTracObject *t_TracObj, QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Simulation"),
              true, parent)
{

}

TaskBox_SimulationManager::~TaskBox_SimulationManager() { delete m_ui; }

//Robot::SimulationManager *TaskBox_SimulationManager::getSimulationManager() {
//  return m_SimulationManager.get();
//}

void TaskBox_SimulationManager::initSimulationPanel() {
//  m_ui->Button_Simulation->setEnabled(true);
  m_proxy->show();
}

void TaskBox_SimulationManager::hidePanel() { m_proxy->hide(); }

void TaskBox_SimulationManager::initSimulationProcess(
    App::DocumentObject *objPtr) {
  if (objPtr == nullptr)
    return;
  initSimulationPanel();
}

void TaskBox_SimulationManager::initUi() {
  m_proxy = new QWidget(this);
  m_ui = new Ui_TaskBox_SimulationManager;
  m_ui->setupUi(m_proxy);
  this->groupLayout()->addWidget(m_proxy);
  setTimer();
  this->resize(m_proxy->width(), m_proxy->height());
}

void TaskBox_SimulationManager::setTimer() {
  m_timer = new QTimer(this);
  m_timer->setInterval(1);
  QObject::connect(m_timer, SIGNAL(timeout()), this, SLOT(timerDone()));
  m_timerPos = 0.0f;
}

void TaskBox_SimulationManager::updateTimeDisplay() {
//  m_ui->textEdit_timeDisplay->setText(
//      QString::fromStdString(calcuteDisplayTime()));
}

void TaskBox_SimulationManager::disableSimulationPanel() {
//  m_ui->timeSlider->setEnabled(false);
//  m_ui->Button_Reset->setEnabled(false);
//  m_ui->Button_End->setEnabled(false);
//  m_ui->Button_Simulation->setEnabled(false);
  m_proxy->hide();
}

void TaskBox_SimulationManager::enableSimulationPanel() {
//  m_ui->timeSlider->setEnabled(true);
//  m_ui->Button_Reset->setEnabled(true);
//  m_ui->Button_End->setEnabled(true);
}

void TaskBox_SimulationManager::changeSimStep(double multi_factor) {
//  m_SimulationManager->changeSimStep(multi_factor);
}

void TaskBox_SimulationManager::changeVisibility() {
//    if(m_ui->checkBox_HideTrajectory->isChecked())
//       m_SimulationManager->changeSimObjectVisiblity(true);
//    else
//       m_SimulationManager->changeSimObjectVisiblity(false);

//      m_SimulationManager->changeSimObjectVisiblity(hide);
}

void TaskBox_SimulationManager::resetSimulationStatus() {
  // Reset Simulator: Panel & Timer
  m_timer->stop();
  m_timerPos = 0;
//  m_ui->Button_Simulation->setText(tr("Start"));
//  m_ui->textEdit_timeDisplay->setText(QObject::tr("0.0s"));
//  m_ui->timeSlider->setValue(1);
//  m_SimulationManager->updateSimulationTargetStatus(0);
}

const std::string TaskBox_SimulationManager::calcuteDisplayTime() {
  if (m_timerPos < 60)
    return string(QString::number(m_timerPos, 'g', 2).toStdString() +
                  string("s"));
  // Longer than 1 hour
  else if (m_timerPos > 3600) {
    auto hour_num = int(m_timerPos / 3600);
    auto min_num = int(m_timerPos - hour_num * 3600) / 60;
    auto sec_num = int(m_timerPos - hour_num * 3600 - min_num * 60);
    return string(std::to_string(hour_num) + string("h ") +
                  std::to_string(min_num) + string("m ") +
                  QString::number(sec_num, 'g', 2).toStdString() +
                  string("s "));
  }
  // Within 1 hour
  else {
    auto min_num = int(m_timerPos / 60);
    auto sec_num = int(m_timerPos - min_num * 60.0);
    return string(std::to_string(min_num) + string("m ") +
                  QString::number(sec_num, 'g', 2).toStdString() + string("s"));
  }
}

void TaskBox_SimulationManager::timerDone() {
//  if (m_timerPos < m_SimulationManager->getSimulationTotalTime()) {
//    m_timerPos += m_SimulationManager->getSimulationStpLen();
//    auto ratio = m_timerPos / m_SimulationManager->getSimulationTotalTime();
//    m_ui->timeSlider->setValue(int((ratio)*m_slider_Maximum));
//    m_timer->start();
//  } else {
////    m_timer->stop();
////    simulation_UnderGoing = false;
////    RD_SetupGui::Gui_Utility_Doc::setSceneGraphSelectable(true);
////    m_ui->Button_Simulation->setText(tr("Start"));
//      stop();
//  }
}

void TaskBox_SimulationManager::pause() {
    m_timer->stop();
//    m_ui->Button_Simulation->setText(tr("Start"));
    simulation_UnderGoing = false;
    Signal_SimulationStarted(false);
    RobotGui::DocUtility::setSceneGraphSelectable(true);
}

void TaskBox_SimulationManager::stop() {
  m_timer->stop();
  simulation_UnderGoing = false;
//  m_timerPos = m_SimulationManager->getSimulationTotalTime();
//  //    updateSimulationTargetStatus(m_timerPos);
//  m_ui->timeSlider->setValue(
//              int((m_timerPos / m_SimulationManager->getSimulationTotalTime()) *
//                  m_slider_Maximum));
//  m_ui->Button_Simulation->setText(tr("Start"));
  RobotGui::DocUtility::setSceneGraphSelectable(true);
  Signal_SimulationStarted(false);
}

void TaskBox_SimulationManager::simulationButtonClicked() {
//  if (!simulation_UnderGoing) {
//    // Start from begining
//    if (std::abs(m_timerPos - m_SimulationManager->getSimulationTotalTime()) <
//        0.5) {
//      // Get Total simualtion time
//      if (!m_SimulationManager->calculateSimulationTime()) {
//        Base::Console().Message(
//            "Get Simulation Time failed! unable to simulate\n");
//        disableSimulationPanel();
//        return;
//      }
//      enableSimulationPanel();
//      m_ui->timeSlider->setValue(0);
//    }

//    m_ui->Button_Simulation->setText(tr("Pause"));
//    simulation_UnderGoing = true;
//    m_timer->start();
//    Signal_SimulationStarted(true);
//  } else {
//    pause();
//  }

  RobotGui::DocUtility::setSceneGraphSelectable(false);
}

void TaskBox_SimulationManager::slider_valueChanged(int value) {
//  m_ui->timeSlider->blockSignals(true);
  auto ratio = float(value / m_slider_Maximum);
//  m_timerPos = ratio * m_SimulationManager->getSimulationTotalTime();
//  m_SimulationManager->updateSimulationTargetStatus(m_timerPos);
  updateTimeDisplay();
//  m_ui->timeSlider->blockSignals(false);
}

#include "TaskManage/moc_TaskBox_SimulationManager.cpp"
