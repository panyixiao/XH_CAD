// Created By Yixiao 2022-09-24

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <QFileDialog>
#include <map>

#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>

#include "QDialogCalibrationPanel.h"
#include "ui_QDialogCalibrationPanel.h"

#include "Mod/Robot/App/Utilites/FileIO_Utility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/App/Trac/RobotWaypoint.h"

#define MaxiumTaskBoxWidth 350

using namespace RobotGui;


QDialogCalibrationPanel::QDialogCalibrationPanel(App::DocumentObject *t_Target,
                                                 QWidget *parent):
    QDialog(parent)
{
    if(t_Target == nullptr || !t_Target->isDerivedFrom(Robot::MechanicDevice::getClassTypeId()))
        return;
    m_TargetObj = static_cast<Robot::MechanicDevice*>(t_Target);
    initUi();
}

QDialogCalibrationPanel::~QDialogCalibrationPanel()
{
    delete m_ui;
}

void QDialogCalibrationPanel::initUi()
{
    m_ui = new Ui_QDialogCalibrationPanel();
    m_ui->setupUi(this);
    for(auto t_Name : m_TargetObj->getAxisNames())
        m_ui->comboBox_axisList->addItem(QString::fromStdString(t_Name));

    QObject::connect(m_ui->comboBox_axisList, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_changeTargetAxis()));
    QObject::connect(m_ui->pushButton_browseFile, SIGNAL(clicked(bool)),
                     this, SLOT(slot_selectCalibrationFile()));
    QObject::connect(m_ui->spinBox_FirstID, SIGNAL(valueChanged(int)),
                     this, SLOT(slot_setDataSegment()));
    QObject::connect(m_ui->spinBox_LastID, SIGNAL(valueChanged(int)),
                     this, SLOT(slot_setDataSegment()));
    QObject::connect(m_ui->pushButton_CalculateCalibrationData, SIGNAL(clicked(bool)),
                     this, SLOT(slot_updateCalibrationResult()));
    QObject::connect(m_ui->pushButton_applyCalibration, SIGNAL(clicked(bool)),
                     this, SLOT(slot_applyToTargetObject()));
    QObject::connect(m_ui->pushButton_DoneCalibration, SIGNAL(clicked(bool)),
                     this, SLOT(slot_DoneCalibration()));

    QObject::connect(m_ui->checkBox_txEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));
    QObject::connect(m_ui->checkBox_tyEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));
    QObject::connect(m_ui->checkBox_tzEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));
    QObject::connect(m_ui->checkBox_rxEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));
    QObject::connect(m_ui->checkBox_ryEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));
    QObject::connect(m_ui->checkBox_rzEnable, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_updateEnableStatus()));

    QObject::connect(m_ui->radioButton_rotationJoint, SIGNAL(clicked(bool)),
                     this, SLOT(slot_jointTypeChanged()));
    QObject::connect(m_ui->radioButton_prismaticJoint, SIGNAL(clicked(bool)),
                     this, SLOT(slot_jointTypeChanged()));
    slot_changeTargetAxis();
    auto path_CalibFile = m_TargetObj->Path_CalibFile.getStrValue();
    if(!path_CalibFile.empty()){
        m_ui->lineEdit->setText(QString::fromStdString(path_CalibFile));
        readinCalibrationFile(QString::fromStdString(path_CalibFile));
    }
}

void QDialogCalibrationPanel::slot_changeTargetAxis()
{
    auto t_AxisID = m_ui->comboBox_axisList->currentIndex();
    m_ui->checkBox_txEnable->setChecked(t_AxisID == 0);
    m_ui->checkBox_tzEnable->setChecked(t_AxisID == 0);
    m_ui->checkBox_tyEnable->setChecked(t_AxisID == 1);
}

void QDialogCalibrationPanel::slot_selectCalibrationFile()
{
    QString fileName = QFileDialog::getOpenFileName(NULL,tr("Calib Data Selection"),tr("/home"),tr("program(*.prg *.txt)"));
    if(fileName.isEmpty()){
        return;
    }
    m_ui->lineEdit->setText(fileName);
    readinCalibrationFile(fileName);
}

void QDialogCalibrationPanel::slot_setDataSegment()
{
    start_ID = m_ui->spinBox_FirstID->value();
    m_ui->spinBox_LastID->setMinimum(start_ID);
    finish_ID = m_ui->spinBox_LastID->value();

    auto startWP_ptr = m_CalibProgramPtr->getWaypointData()[start_ID];
    m_ui->label_startPntInfo->setText(QString::fromStdString(startWP_ptr->getWP_Name()));

    auto finishWP_ptr = m_CalibProgramPtr->getWaypointData()[finish_ID];
    m_ui->label_finishPntInfo->setText(QString::fromStdString(finishWP_ptr->getWP_Name()));
}

void QDialogCalibrationPanel::slot_updateCalibrationResult()
{
    auto distance = finish_ID - start_ID + 1;
    if(distance%3!=0){
        Base::Console().Warning("TaskBoxPositionerSetupPanel: Segment Data cannot be divided by 3!");
        return;
    }
    m_CalibPoses.clear();
    for(int i = start_ID; i<=finish_ID; i++){
       m_CalibPoses.push_back(m_CalibProgramPtr->getWaypointData()[i]->getWPCartPose_GP1());
    }

    Base::Placement t_Center;
    if(flag_isRotationJoint){
        t_Center = Robot::DS_Utility::calculateTargetCalibArcCenterPose(m_CalibPoses);
    }

    m_ui->label_result_Tx->setText(QString::number(t_Center.getPosition().x));
    m_ui->label_result_Ty->setText(QString::number(t_Center.getPosition().y));
    m_ui->label_result_Tz->setText(QString::number(t_Center.getPosition().z));
    double y,p,r;
    t_Center.getRotation().getYawPitchRoll(y,p,r);
    m_ui->label_result_Rz->setText(QString::number(y));
    m_ui->label_result_Ry->setText(QString::number(p));
    m_ui->label_result_Rx->setText(QString::number(r));


}

void QDialogCalibrationPanel::slot_applyToTargetObject()
{
    if(m_TargetObj == nullptr)
        return;
    auto originPose = m_TargetObj->Pose_Ref2Base.getValue();

    double new_Tx = m_ui->label_result_Tx->isEnabled()?m_ui->label_result_Tx->text().toDouble():originPose.getPosition().x;
    double new_Ty = m_ui->label_result_Ty->isEnabled()?m_ui->label_result_Ty->text().toDouble():originPose.getPosition().y;
    double new_Tz = m_ui->label_result_Tz->isEnabled()?m_ui->label_result_Tz->text().toDouble():originPose.getPosition().z;
    double yaw, pit, roll;
    originPose.getRotation().getYawPitchRoll(yaw, pit, roll);
    double new_Rx = m_ui->label_result_Rx->isEnabled()?m_ui->label_result_Rx->text().toDouble():roll;
    double new_Ry = m_ui->label_result_Ry->isEnabled()?m_ui->label_result_Ry->text().toDouble():pit;
    double new_Rz = m_ui->label_result_Rz->isEnabled()?m_ui->label_result_Rz->text().toDouble():yaw;
    Base::Rotation new_Rot;
    new_Rot.setYawPitchRoll(new_Rz,new_Ry,new_Rx);
    m_TargetObj->Pose_Ref2Base.setValue(Base::Placement(Base::Vector3d(new_Tx,new_Ty,new_Tz),
                                                        new_Rot));
    Q_EMIT signal_updatePosePanel();
}

void QDialogCalibrationPanel::slot_DoneCalibration()
{
    this->close();
}

void QDialogCalibrationPanel::slot_jointTypeChanged()
{
    if(m_ui->radioButton_rotationJoint->isChecked()){
        m_ui->groupBox_jointMovingAxis->setTitle(tr("Rotation Axis"));
        flag_isRotationJoint = true;
    }
    else if(m_ui->radioButton_prismaticJoint->isChecked()){
        m_ui->groupBox_jointMovingAxis->setTitle(tr("Translate Axis"));
        flag_isRotationJoint = false;
    }
}

void QDialogCalibrationPanel::slot_updateEnableStatus()
{
    m_ui->label_result_Tx->setEnabled(m_ui->checkBox_txEnable->isChecked());
    m_ui->label_result_Ty->setEnabled(m_ui->checkBox_tyEnable->isChecked());
    m_ui->label_result_Tz->setEnabled(m_ui->checkBox_tzEnable->isChecked());

    m_ui->checkBox_rxEnable->setChecked(m_ui->checkBox_txEnable->isChecked());
    m_ui->checkBox_ryEnable->setChecked(m_ui->checkBox_tyEnable->isChecked());
    m_ui->checkBox_rzEnable->setChecked(m_ui->checkBox_tzEnable->isChecked());
    m_ui->label_result_Rx->setEnabled(m_ui->checkBox_rxEnable->isChecked());
    m_ui->label_result_Ry->setEnabled(m_ui->checkBox_ryEnable->isChecked());
    m_ui->label_result_Rz->setEnabled(m_ui->checkBox_rzEnable->isChecked());
}

bool QDialogCalibrationPanel::readinCalibrationFile(const QString& t_path)
{
    auto t_ProgramPtr = FileIO_Utility::readinProgramFromDrive(t_path.toStdString(),
                                                               std::string());
    if(t_ProgramPtr == nullptr || !t_ProgramPtr->isProgramValid()){
        Base::Console().Warning("TaskBoxPositionerSetupPanel: Failed to reading calibration Data");
        return false;
    }
    m_CalibProgramPtr = t_ProgramPtr;
    m_CalibPoses.clear();
    auto t_size = m_CalibProgramPtr->getWaypointData().size();

    m_ui->spinBox_FirstID->setMinimum(1);
    m_ui->spinBox_FirstID->setMaximum(t_size-1);
    m_ui->spinBox_LastID->setMinimum(3);
    m_ui->spinBox_LastID->setMaximum(t_size-1);

    m_TargetObj->Path_CalibFile.setValue(t_path.toStdString().c_str());
    return true;
}


#include "moc_QDialogCalibrationPanel.cpp"
