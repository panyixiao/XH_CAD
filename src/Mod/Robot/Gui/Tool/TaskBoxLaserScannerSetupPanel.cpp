#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxLaserScannerSetupPanel.h"

#include <Eigen/Dense>
#include <Base/Console.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <QPushButton>

#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/App/Utilites/CAD_Utility.h>
#include <Mod/Robot/Gui/Tool/ViewProviderScannerObject.h>
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxLaserScannerSetupPanel.h"

using namespace RobotGui;

TaskBoxLaserScannerSetupPanel::TaskBoxLaserScannerSetupPanel(Robot::ToolObject *t_toolPtr,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_FaceSelection,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_EdgeSelection)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),tr("Scanner Tool Setup")) {
  if (t_toolPtr == nullptr || t_EdgeSelection == nullptr || t_FaceSelection == nullptr)
    return;

  m_DocPtr = t_toolPtr->getDocument();
  m_ScannerPtr = static_cast<Robot::ScannerObject*>(t_toolPtr);
  m_FaceRef = t_FaceSelection;
  m_EdgeRef = t_EdgeSelection;
  initUi();
}

void TaskBoxLaserScannerSetupPanel::mouseReleaseEvent(QMouseEvent *ev)
{
}

bool TaskBoxLaserScannerSetupPanel::accept() {
//    t_Dragger->destroyDragger();
    return true;
}

bool TaskBoxLaserScannerSetupPanel::reject() {
//    t_Dragger->destroyDragger();
    return true;
}

void TaskBoxLaserScannerSetupPanel::initUi() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxLaserScannerSetupPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  initUi_AssembleWidgets();

  QObject::connect(m_ui->pushButton_Assemble, SIGNAL(clicked()),
                   this, SLOT(slot_assembleTool2Robot()));

  initUi_PoseBox();
  initUi_VisualizeBox();
  initUi_CalibBox();

  QObject::connect(m_ui->pushButton_finishSetup, SIGNAL(clicked()),
                   this, SLOT(slot_finishSetup()));
}

void TaskBoxLaserScannerSetupPanel::initUi_PoseBox()
{

    QObject::connect(m_ui->radioButton_Base, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    QObject::connect(m_ui->radioButton_Lens, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    QObject::connect(m_ui->radioButton_front,SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    m_ui->radioButton_Base->setChecked(true);

    QObject::connect(m_ui->doubleSpinBox_Sensor_tX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_tY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_tZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->pushButton_CalcCenter, SIGNAL(clicked()),
                     this, SLOT(slot_getSelectedCenter()));
}

void TaskBoxLaserScannerSetupPanel::initUi_VisualizeBox()
{
    QObject::connect(m_ui->pushButton_LaserOn, SIGNAL(clicked()),
                     this, SLOT(slot_setLaserOn()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_distance, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeScanRange()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_amplitude, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeScanRange()));
}

void TaskBoxLaserScannerSetupPanel::initUi_CalibBox()
{
    QObject::connect(m_ui->pushButton_calibLensPose, SIGNAL(clicked()),
                     this, SLOT(slot_calibLenPose()));
}

void TaskBoxLaserScannerSetupPanel::blockPosePanelSignals(bool block)
{
    m_ui->doubleSpinBox_Sensor_tX->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_tY->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_tZ->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rX->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rY->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rZ->blockSignals(block);
}

void TaskBoxLaserScannerSetupPanel::updatePoseBox(const Base::Placement& c_Pose)
{
    blockPosePanelSignals(true);
    m_ui->doubleSpinBox_Sensor_tX->setValue(c_Pose.getPosition().x);
    m_ui->doubleSpinBox_Sensor_tY->setValue(c_Pose.getPosition().y);
    m_ui->doubleSpinBox_Sensor_tZ->setValue(c_Pose.getPosition().z);
    double y,p,r;
    c_Pose.getRotation().getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_Sensor_rX->setValue(r);
    m_ui->doubleSpinBox_Sensor_rY->setValue(p);
    m_ui->doubleSpinBox_Sensor_rZ->setValue(y);

    blockPosePanelSignals(false);
}

void TaskBoxLaserScannerSetupPanel::enablePoseBox(bool flag)
{
    m_ui->doubleSpinBox_Sensor_tX->setEnabled(flag);
    m_ui->doubleSpinBox_Sensor_tY->setEnabled(flag);
    m_ui->doubleSpinBox_Sensor_tZ->setEnabled(flag);
    m_ui->doubleSpinBox_Sensor_rX->setEnabled(flag);
    m_ui->doubleSpinBox_Sensor_rY->setEnabled(flag);
    m_ui->doubleSpinBox_Sensor_rZ->setEnabled(flag);
}

void TaskBoxLaserScannerSetupPanel::initUi_AssembleWidgets()
{
    if(m_ScannerPtr->isFloating()){
        m_ui->comboBox_RobotList->setEnabled(true);
        m_ui->comboBox_RobotList->clear();
        auto objList = m_DocPtr->getObjects();
        for(auto objPtr : objList){
            if(objPtr->isDerivedFrom(Robot::Robot6AxisObject::getClassTypeId())){
                if(!static_cast<Robot::Robot6AxisObject*>(objPtr)->ScannerAssembled()){
                    m_ui->comboBox_RobotList->addItem(tr(objPtr->getNameInDocument()));
                }
            }
        }
        m_ui->pushButton_Assemble->setText(tr("Install"));
    }
    else{
        m_ui->comboBox_RobotList->addItem(tr(m_ScannerPtr->ParentRobotName.getValue()));
        m_ui->comboBox_RobotList->setEnabled(false);
        m_ui->pushButton_Assemble->setText(tr("Uninstall"));
    }
}

void TaskBoxLaserScannerSetupPanel::slot_getSelectedCenter()
{
    if(m_FaceRef==nullptr)
        return;
    Base::Placement featuerCenter;
    if(m_FaceRef->isSelectionValid()){
        m_FaceRef->sendSelection2Property();
    }
    if(m_EdgeRef->isSelectionValid()){
        m_EdgeRef->sendSelection2Property();
    }

    featuerCenter = m_ScannerPtr->calculateSelectedFeatureCenter();

    if(m_ui->radioButton_Base->isChecked()){
        m_ScannerPtr->setNewMountOrigin(featuerCenter);
        updatePoseBox(m_ScannerPtr->Trans_O2M.getValue());
    }
    else if(m_ui->radioButton_Lens->isChecked()){
        m_ScannerPtr->setNewTipPosition(featuerCenter);
        updatePoseBox(m_ScannerPtr->Trans_M2T.getValue());
    }

    Q_EMIT Signal_updateDraggerPose(featuerCenter);
}

void TaskBoxLaserScannerSetupPanel::slot_assembleTool2Robot()
{
    if(m_ScannerPtr->ParentRobotName.isEmpty()){
        auto selectedRobotName = m_ui->comboBox_RobotList->currentText().toStdString();
        m_ScannerPtr->assembleToRobot(selectedRobotName);
    }
    else{
        m_ScannerPtr->detachToolFromRobot();
    }
    initUi_AssembleWidgets();
}

void TaskBoxLaserScannerSetupPanel::slot_updatePose()
{
    assert(m_ScannerPtr!=nullptr);
    Base::Placement newPose;
    newPose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_Sensor_tX->value(),
                                       m_ui->doubleSpinBox_Sensor_tY->value(),
                                       m_ui->doubleSpinBox_Sensor_tZ->value()));
    Base::Rotation newRot;
    newRot.setYawPitchRoll(m_ui->doubleSpinBox_Sensor_rZ->value(),
                           m_ui->doubleSpinBox_Sensor_rY->value(),
                           m_ui->doubleSpinBox_Sensor_rX->value());
    newPose.setRotation(newRot);

    Base::Placement displayPose;
    if(m_ui->radioButton_Base->isChecked()){
        m_ScannerPtr->Trans_O2M.setValue(newPose);
        displayPose = m_ScannerPtr->Pose_Mount.getValue();
    }
    else if(m_ui->radioButton_Lens->isChecked()){
        m_ScannerPtr->Trans_M2T.setValue(newPose);
        displayPose = m_ScannerPtr->getPose_ABSToolTip();
    }
    else if(m_ui->radioButton_front->isChecked()){
        m_ScannerPtr->Trans_M2T.setValue(newPose);
        displayPose = m_ScannerPtr->getPose_ABSToolFront();
    }
    Q_EMIT Signal_updateDraggerPose(displayPose);
}

void TaskBoxLaserScannerSetupPanel::slot_setTatgetChanged()
{
    Base::Placement t_Pose;
    if(m_ui->radioButton_Base->isChecked()){
        enablePoseBox(true);
        t_Pose = m_ScannerPtr->Trans_O2M.getValue();
        Q_EMIT Signal_updateDraggerPose(m_ScannerPtr->Pose_Mount.getValue());
    }
    else if(m_ui->radioButton_Lens->isChecked()){
        enablePoseBox(true);
        t_Pose = m_ScannerPtr->Trans_M2T.getValue();
        Q_EMIT Signal_updateDraggerPose(m_ScannerPtr->getPose_ABSToolTip());
    }
    else if(m_ui->radioButton_front->isChecked()){
        enablePoseBox(false);
        t_Pose = m_ScannerPtr->getTransform_Mount2Front();
        Q_EMIT Signal_updateDraggerPose(m_ScannerPtr->getPose_ABSToolFront());
    }
    updatePoseBox(t_Pose);
}


void TaskBoxLaserScannerSetupPanel::slot_setLaserOn()
{
    if(!m_ScannerPtr->LaserOn.getValue()){
        m_ScannerPtr->LaserOn.setValue(true);
        m_ui->pushButton_LaserOn->setText(tr("Turn Off"));
    }
    else{
        m_ScannerPtr->LaserOn.setValue(false);
        m_ui->pushButton_LaserOn->setText(tr("Turn On"));
    }
}

void TaskBoxLaserScannerSetupPanel::slot_changeScanRange()
{
    m_ScannerPtr->ScanDistance.setValue(m_ui->doubleSpinBox_Sensor_distance->value());
    m_ScannerPtr->ScanAmplitute.setValue(m_ui->doubleSpinBox_Sensor_amplitude->value());
}

void TaskBoxLaserScannerSetupPanel::slot_calibLenPose()
{
    Base::Placement P1,P2,P3,P4;
    P1.setPosition(Base::Vector3d(m_ui->doubleSpinBox_P1_tx->value(),
                                  m_ui->doubleSpinBox_P1_ty->value(),
                                  m_ui->doubleSpinBox_P1_tz->value()));

    P2.setPosition(Base::Vector3d(m_ui->doubleSpinBox_P2_tx->value(),
                                  m_ui->doubleSpinBox_P2_ty->value(),
                                  m_ui->doubleSpinBox_P2_tz->value()));

    P3.setPosition(Base::Vector3d(m_ui->doubleSpinBox_P3_tx->value(),
                                  m_ui->doubleSpinBox_P3_ty->value(),
                                  m_ui->doubleSpinBox_P3_tz->value()));

    P4.setPosition(Base::Vector3d(m_ui->doubleSpinBox_P4_tx->value(),
                                  m_ui->doubleSpinBox_P4_ty->value(),
                                  m_ui->doubleSpinBox_P4_tz->value()));

    // https://blog.csdn.net/hunter_wwq/article/details/41044179
    Base::Vector3d vec_1 = P3.getPosition() - P1.getPosition();
    Base::Vector3d vec_2 = P4.getPosition() - P2.getPosition();
    auto v1 = vec_1.Normalize();
    auto v2 = vec_2.Normalize();

    auto v_12 = P1.getPosition() - P2.getPosition();
    auto t1 = DS_Utility::crossProduct(v_12,v2).Length()/DS_Utility::crossProduct(v2,v1).Length();

    auto Pi = P1.getPosition() + t1*v1;
    auto new_Z = (v1 + v2).Normalize();
    auto new_Y = DS_Utility::crossProduct(v1,v2).Normalize();
    auto new_X = DS_Utility::crossProduct(new_Y,new_Z);



    // Should be Robot Base
    Base::Vector3d ref_Z = Base::Vector3d(0,0,1);
    auto Rot_Axis = DS_Utility::crossProduct(new_Z,ref_Z).Normalize();
    auto cos_Angle = DS_Utility::dotProduct(ref_Z,new_Z)/(ref_Z.Length()*new_Z.Length());
    auto rot_Angle = std::acos(cos_Angle);
    auto sin_Angle = std::sin(rot_Angle);

    auto angle_Deg = rot_Angle*180/M_PI;


    Eigen::Matrix3d t_Eye = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_RotVec(Rot_Axis.x,Rot_Axis.y,Rot_Axis.z);
    Eigen::Matrix3d mat;
    mat<<          0, -t_RotVec(2), t_RotVec(1),
         t_RotVec(2),            0, t_RotVec(0),
        -t_RotVec(1),  t_RotVec(0),           0;

    //罗德里斯公式（Rodriguez formula） https://blog.csdn.net/open123852/article/details/106430073/
    Eigen::Matrix3d Mat_Rodriguez = cos_Angle*t_Eye + (1 - cos_Angle)*DS_Utility::vectorProduct(t_RotVec,t_RotVec) + sin_Angle*mat;

//    Base::Matrix4D t_Origin =   Base::Matrix4D (Mat_Rodriguez(0,0),Mat_Rodriguez(0,1),Mat_Rodriguez(0,2), Pi.x,
//                                                Mat_Rodriguez(1,0),Mat_Rodriguez(1,1),Mat_Rodriguez(1,2), Pi.y,
//                                                Mat_Rodriguez(2,0),Mat_Rodriguez(2,1),Mat_Rodriguez(2,2), Pi.z,
//                                                0.0               ,               0.0,               0.0,    1);
    Base::Matrix4D t_Origin = Base::Matrix4D(new_X.x, new_Y.x, new_Z.x, Pi.x,
                                             new_X.y, new_Y.y, new_Z.y, Pi.y,
                                             new_X.z, new_Y.z, new_Z.z, Pi.z,
                                                 0.0,     0.0,     0.0,    1);


    Base::Placement flanPose;
    flanPose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_Flan_tx->value(),
                                        m_ui->doubleSpinBox_Flan_ty->value(),
                                        m_ui->doubleSpinBox_Flan_tz->value()));
    Base::Rotation t_Rot;
    t_Rot.setYawPitchRoll(m_ui->doubleSpinBox_Flan_rz->value(),
                          m_ui->doubleSpinBox_Flan_ry->value(),
                          m_ui->doubleSpinBox_Flan_rx->value());
    flanPose.setRotation(t_Rot);

    auto T_Trans = Base::Placement(flanPose.inverse().toMatrix() * t_Origin);
    double yaw,pitch,roll;
    T_Trans.getRotation().getYawPitchRoll(yaw,pitch,roll);

    m_ui->radioButton_Lens->setChecked(true);
    m_ui->doubleSpinBox_Sensor_tX->setValue(T_Trans.getPosition().x);
    m_ui->doubleSpinBox_Sensor_tY->setValue(T_Trans.getPosition().y);
    m_ui->doubleSpinBox_Sensor_tZ->setValue(T_Trans.getPosition().z);
    m_ui->doubleSpinBox_Sensor_rZ->setValue(yaw);
    m_ui->doubleSpinBox_Sensor_rY->setValue(pitch);
    m_ui->doubleSpinBox_Sensor_rX->setValue(roll);
}

void TaskBoxLaserScannerSetupPanel::slot_finishSetup()
{
    Q_EMIT Signal_finishSetup();
}


#include "moc_TaskBoxLaserScannerSetupPanel.cpp"
