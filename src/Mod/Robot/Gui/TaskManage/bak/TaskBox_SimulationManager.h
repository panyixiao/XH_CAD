// Created By Yixiao 2022-05-10
#pragma once

#include <QObject>
#include <QStringListModel>
#include <QWidget>

#include <App/DocumentObject.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/ViewProviderDocumentObject.h>

#include "Mod/Robot/App/TaskManage/TaskObject.h"
#include "Mod/Robot/App/TaskManage/SimulationManager.h"
#include "Mod/Robot/App/Trac/RobotTracObject.h"

class Ui_TaskBox_SimulationManager;

namespace RobotGui {

class TaskBox_SimulationManager : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_SimulationManager(Robot::TaskObject* t_Object, QWidget *parent = 0);
  TaskBox_SimulationManager(Robot::RobotTracObject* t_TracObj, QWidget* parent = 0);
  ~TaskBox_SimulationManager();

  Q_SIGNAL void Signal_SimulationStarted(bool started);

  void initSimulationPanel();
  void hidePanel();

public Q_SLOTS:
  void initSimulationProcess(App::DocumentObject *objPtr = nullptr);

private Q_SLOTS:
  void simulationButtonClicked();
  void pause();
  void stop();
  void timerDone();
  void slider_valueChanged(int value);
  void changeVisibility();

  void resetSimulationStatus();
  void changeSimStep(double multi_factor);

protected:
  void initUi();
  void setTimer();
  void updateTimeDisplay();
  void disableSimulationPanel();
  void enableSimulationPanel();
  const std::string calcuteDisplayTime();

protected:
  QTimer *m_timer;
  float m_timerPos;

private:
  QWidget *m_proxy;
  Ui_TaskBox_SimulationManager *m_ui;
  App::Document *m_currentDoc;
  Robot::TaskObject* m_TaskObjectPtr = nullptr;
  bool simulation_UnderGoing = false;
  float m_slider_Maximum = 1000.0;
};
}
