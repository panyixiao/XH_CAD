
#ifndef GUI_TaskBox_TracSimulator_H
#define GUI_TaskBox_TracSimulator_H

#include <Base/UnitsApi.h>


#include <Gui/TaskView/TaskView.h>
#include <Gui/Selection.h>

//#include <Mod/Robot/App/RobotObject.h>

#include <QStringList>
#include <QStringListModel>
#include <QItemSelection>

#include <Mod/Robot/App/Mechanics/KinematicModel.h>
//#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
//#include "Mod/Robot/Gui/Mechanics/ViewProviderRobot6AxisObject.h"
//#include <Mod/Robot/App/Trac/RobotTracObject.h>
#include <Mod/Robot/App/TaskManage/TaskObject.h>
#include <Mod/Robot/App/TaskManage/ProgramSimulator.h>


class Ui_TaskBox_TracSimulator;

namespace App {
class Property;
}

namespace Gui {
class ViewProvider;
}

namespace RobotGui {


class TaskBox_TracSimulator : public Gui::TaskView::TaskBox
{
    Q_OBJECT

public:
//    TaskBox_TracSimulator(Robot::RobotTracObject *t_TracObj,
//                          QWidget *parent = 0);
    TaskBox_TracSimulator(Robot::TaskObject *t_TaskObj,
                          QWidget *parent = 0);
//    bool updateSimulationTarget(Robot::RobotTracObject* t_TracObj);
    bool updateSimulationTarget(Robot::TaskObject* t_TaskObj);
    ~TaskBox_TracSimulator();
    /// Observer message from the Selection
    void OnChange(Gui::SelectionSingleton::SubjectType &rCaller,
                  Gui::SelectionSingleton::MessageType Reason);
    void updateTableDisplay(bool showTable = true);

    Q_SIGNAL void Signal_expdSimulationPanel();
    Q_SIGNAL void Signal_foldSimulationPanel();
    Q_SIGNAL void Signal_simulationOn(bool);

public Q_SLOTS:
    void slot_updateSimProgram(const RobotProg_sptr t_Program);
    void slot_updateSimProgram(const Robot::TaskObject *t_cTaskObjPtr);

private Q_SLOTS:
    void slot_changePanelSize();
//    void slot_hideSimulationPanel();
    void slot_changeSelectedCommand(const QItemSelection &selection);
    void slot_runButtonClicked(void);
    void slot_setToStart(void);
    void slot_setToEnd(void);

    void timerDone(void);

protected:
//    void updateSimulator(void);
    void updateCommandList();
    bool updateSimualtionTrac();
    void stopSimulation();
    void initUi(bool showCmdlineWidget = false);
    void generateCommandExecutingMsg(const size_t t_CmdID);

    QTimer *timer;
    App::Document* m_DocPtr = nullptr;
    Robot::ProgramSimulator    *m_SimulatorPtr = nullptr;
    Robot::TaskObject       *m_TaskObjPtr = nullptr;

//    Robot::RobotTracObject  *m_TracObjPtr = nullptr;
    QStringList *m_stringList = nullptr;
    QStringListModel *m_stringListModel = nullptr;

    int c_CmdID = 0;
    int temp_CmdID = 0;
    int L_ID = 0;

    bool flag_block = false;

    float timePos = 0.0;
    float totalTime = 0.0;
    float duration = 0.0;

private:
    QWidget* proxy;
    Ui_TaskBox_TracSimulator* m_ui;
    bool panel_expanded = false;
    bool flag_paused = true;
};

} //namespace PartDesignGui

#endif // GUI_TASKVIEW_TASKAPPERANCE_H
