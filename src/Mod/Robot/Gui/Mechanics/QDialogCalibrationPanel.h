// Create by Yixiao 2022/09/25

#ifndef QDIALOG_CALIBRATIONPANEL_H
#define QDIALOG_CALIBRATIONPANEL_H

#include <QDialog>
#include <App/DocumentObject.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/TaskManage/RobotProgram.h>
#include "Mod/Robot/App/Mechanics/MechanicBase.h"

class Ui_QDialogCalibrationPanel;
namespace RobotGui {

class QDialogCalibrationPanel : public QDialog{
    Q_OBJECT
public:
    QDialogCalibrationPanel(App::DocumentObject* t_Tatget,QWidget *parent = 0);
    ~QDialogCalibrationPanel();

Q_SIGNALS:
  void signal_updatePosePanel();

protected:
    void initUi();

private Q_SLOTS:
    void slot_changeTargetAxis();
    void slot_selectCalibrationFile();
    void slot_setDataSegment();
    void slot_updateCalibrationResult();
    void slot_applyToTargetObject();
    void slot_DoneCalibration();
    void slot_jointTypeChanged();
    void slot_updateEnableStatus();

private:
    bool readinCalibrationFile(const QString &t_path);

private:
    QWidget *m_proxy = nullptr;
    Ui_QDialogCalibrationPanel *m_ui = nullptr;
    std::shared_ptr<Robot::RobotProgram> m_CalibProgramPtr;
    std::vector<Base::Placement> m_CalibPoses;
    size_t start_ID;
    size_t finish_ID;
    bool flag_isRotationJoint = true;
    // TODO: Upgrade to MechanicObject*
    Robot::MechanicBase *m_TargetObj;
};


}
#endif // QDIALOG_CALIBRATIONPANEL_H
