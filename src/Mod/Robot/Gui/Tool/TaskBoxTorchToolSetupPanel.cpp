#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxTorchToolSetupPanel.h"

#include <Base/Console.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <QPushButton>

#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/App/Utilites/CAD_Utility.h>
#include <Mod/Robot/Gui/Tool/ViewProviderTorchObject.h>
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxTorchToolSetupPanel.h"

using namespace RobotGui;

TaskBoxTorchToolSetupPanel::TaskBoxTorchToolSetupPanel(Robot::ToolObject *t_toolPtr,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_FaceSelection,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_EdgeSelection)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),tr("Torch Tool Setup")) {
  if (t_toolPtr == nullptr || t_EdgeSelection == nullptr || t_FaceSelection == nullptr)
    return;

  m_DocPtr = t_toolPtr->getDocument();
  m_TorchPtr = static_cast<Robot::TorchObject*>(t_toolPtr);
  m_FaceRef = t_FaceSelection;
  m_EdgeRef = t_EdgeSelection;

  initUi();
}

void TaskBoxTorchToolSetupPanel::mouseReleaseEvent(QMouseEvent *ev)
{
}

void TaskBoxTorchToolSetupPanel::initUi() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxTorchToolSetupPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  initUi_AssembleWidgets();

  QObject::connect(m_ui->pushButton_Assemble, SIGNAL(clicked()),
                   this, SLOT(slot_assembleTool2Robot()));
  initUi_PoseBox();
  initUi_VisualizeBox();
  QObject::connect(m_ui->pushButton_finishSetup, SIGNAL(clicked()),
                   this, SLOT(slot_finishSetup()));
}

void TaskBoxTorchToolSetupPanel::initUi_PoseBox()
{
    QObject::connect(m_ui->radioButton_Base, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    QObject::connect(m_ui->radioButton_Tip, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    m_ui->radioButton_Base->setChecked(true);
//    m_ui->radioButton_Tip->setEnabled(false);

    QObject::connect(m_ui->doubleSpinBox_Torch_tX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_tY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_tZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updatePose()));
    QObject::connect(m_ui->pushButton_CalcCenter, SIGNAL(clicked()),
                     this, SLOT(slot_getSelectedCenter()));
}

void TaskBoxTorchToolSetupPanel::initUi_VisualizeBox()
{
    QObject::connect(m_ui->pushButton_sparkOn, SIGNAL(clicked()),
                     this, SLOT(slot_setsparkOn()));
}

void TaskBoxTorchToolSetupPanel::blockPosePanelSignals(bool block)
{
    m_ui->doubleSpinBox_Torch_tX->blockSignals(block);
    m_ui->doubleSpinBox_Torch_tY->blockSignals(block);
    m_ui->doubleSpinBox_Torch_tZ->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rX->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rY->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rZ->blockSignals(block);
}

void TaskBoxTorchToolSetupPanel::updatePoseBox(const Base::Placement& c_Pose)
{
    blockPosePanelSignals(true);
    m_ui->doubleSpinBox_Torch_tX->setValue(c_Pose.getPosition().x);
    m_ui->doubleSpinBox_Torch_tY->setValue(c_Pose.getPosition().y);
    m_ui->doubleSpinBox_Torch_tZ->setValue(c_Pose.getPosition().z);
    double y,p,r;
    c_Pose.getRotation().getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_Torch_rX->setValue(r);
    m_ui->doubleSpinBox_Torch_rY->setValue(p);
    m_ui->doubleSpinBox_Torch_rZ->setValue(y);

    blockPosePanelSignals(false);
}

void TaskBoxTorchToolSetupPanel::initUi_AssembleWidgets()
{
    if(m_TorchPtr->isFloating()){
        m_ui->comboBox_RobotList->setEnabled(true);
        m_ui->comboBox_RobotList->clear();
        auto objList = m_DocPtr->getObjects();
        for(auto objPtr : objList){
            if(objPtr->isDerivedFrom(Robot::Robot6AxisObject::getClassTypeId())){
                if(!static_cast<Robot::Robot6AxisObject*>(objPtr)->TorchAssembled()){
                    m_ui->comboBox_RobotList->addItem(tr(objPtr->getNameInDocument()));
                }
            }
        }
        m_ui->pushButton_Assemble->setText(tr("Install"));
    }
    else{
        m_ui->comboBox_RobotList->addItem(tr(m_TorchPtr->MountedRobot.getValue()));
        m_ui->comboBox_RobotList->setEnabled(false);
        m_ui->pushButton_Assemble->setText(tr("Uninstall"));
    }
}

void TaskBoxTorchToolSetupPanel::slot_getSelectedCenter()
{
    if(m_FaceRef==nullptr)
        return;
    if(m_FaceRef->isSelectionValid()){
        m_FaceRef->sendSelection2Property();
    }
    if(m_EdgeRef->isSelectionValid()){
        m_EdgeRef->sendSelection2Property();
    }
    Base::Placement selectedCenter = m_TorchPtr->calculateSelectedFeatureCenter();

    if(m_ui->radioButton_Base->isChecked()){
        m_TorchPtr->setNewMountOrigin(selectedCenter);
        updatePoseBox(m_TorchPtr->Trans_O2M.getValue());
//        m_ui->radioButton_Tip->setEnabled(true);
    }
    else if(m_ui->radioButton_Tip->isChecked()){
        m_TorchPtr->setNewTipPosition(selectedCenter);
        updatePoseBox(m_TorchPtr->Trans_M2T.getValue());
    }

    Q_EMIT Signal_updateDraggerPose(selectedCenter);
}

void TaskBoxTorchToolSetupPanel::slot_assembleTool2Robot()
{

    if(m_TorchPtr->MountedRobot.isEmpty()){
        auto selectedRobotName = m_ui->comboBox_RobotList->currentText().toStdString();
        m_TorchPtr->assembleToRobot(selectedRobotName);
    }
    else{
        m_TorchPtr->detachToolFromRobot();
    }
    initUi_AssembleWidgets();
    Q_EMIT Signal_updateDraggerPose(m_TorchPtr->Pose_Mount.getValue());
}

void TaskBoxTorchToolSetupPanel::slot_updatePose()
{
    assert(m_TorchPtr!=nullptr);
    Base::Placement newPose;
    newPose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_Torch_tX->value(),
                                       m_ui->doubleSpinBox_Torch_tY->value(),
                                       m_ui->doubleSpinBox_Torch_tZ->value()));
    Base::Rotation newRot;
    newRot.setYawPitchRoll(m_ui->doubleSpinBox_Torch_rZ->value(),
                           m_ui->doubleSpinBox_Torch_rY->value(),
                           m_ui->doubleSpinBox_Torch_rX->value());
    newPose.setRotation(newRot);

    Base::Placement displayPose;
    if(m_ui->radioButton_Base->isChecked()){
        m_TorchPtr->Trans_O2M.setValue(newPose);
        displayPose = m_TorchPtr->Pose_Mount.getValue();
    }
    else if(m_ui->radioButton_Tip->isChecked()){
        m_TorchPtr->Trans_M2T.setValue(newPose);
        displayPose = m_TorchPtr->getPose_ABSToolTip();
    }

    Q_EMIT Signal_updateDraggerPose(displayPose);
}

void TaskBoxTorchToolSetupPanel::slot_setTatgetChanged()
{
    Base::Placement t_Pose;
    if(m_ui->radioButton_Base->isChecked()){
        t_Pose = m_TorchPtr->Trans_O2M.getValue();
        Q_EMIT Signal_updateDraggerPose(m_TorchPtr->Pose_Mount.getValue());
    }
    else if(m_ui->radioButton_Tip->isChecked()){
        t_Pose = m_TorchPtr->Trans_M2T.getValue();
        Q_EMIT Signal_updateDraggerPose(m_TorchPtr->getPose_ABSToolTip());
    }
    updatePoseBox(t_Pose);
}


void TaskBoxTorchToolSetupPanel::slot_setsparkOn()
{
    if(!m_TorchPtr->SparkOn.getValue()){
        m_TorchPtr->SparkOn.setValue(true);
        m_ui->pushButton_sparkOn->setText(tr("Turn Off"));
    }
    else{
        m_TorchPtr->SparkOn.setValue(false);
        m_ui->pushButton_sparkOn->setText(tr("Turn On"));
    }
}

void TaskBoxTorchToolSetupPanel::slot_changeTorchVisualEffect()
{

}

void TaskBoxTorchToolSetupPanel::slot_finishSetup()
{
    Q_EMIT Signal_finishSetup();
}


#include "moc_TaskBoxTorchToolSetupPanel.cpp"
