#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxTorchToolSetupPanel.h"

#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <QPushButton>

//#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/App/Mechanics/MechanicRobot.h>
#include <Mod/Robot/App/Mechanics/MechanicPoser.h>
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
  if(m_TorchPtr->FilePath_Param.getStrValue().empty())
      creatingMode = true;
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
  if(creatingMode){
      m_ui->pushButton_finishSetup->setText(tr("Save Tool"));
  }else{
      m_ui->lineEdit_brand->setEnabled(false);
      m_ui->lineEdit_brand->setText(tr(m_TorchPtr->ToolBrand.getStrValue().c_str()));
      m_ui->comboBox_tubeType->setEnabled(false);
      m_ui->comboBox_tubeType->setCurrentIndex((int)m_TorchPtr->m_Info.tube_Type);
      m_ui->doubleSpinBox_tubeLength->setEnabled(false);
      m_ui->doubleSpinBox_tubeLength->setValue(m_TorchPtr->m_Info.tube_Length);
      m_ui->pushButton_finishSetup->setText(tr("Finish Edit"));
  }
  QObject::connect(m_ui->pushButton_finishSetup, SIGNAL(clicked()),
                   this, SLOT(slot_finishSetupButtonClicked()));
}

void TaskBoxTorchToolSetupPanel::initUi_PoseBox()
{
    QObject::connect(m_ui->radioButton_Base, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    QObject::connect(m_ui->radioButton_Tip, SIGNAL(clicked(bool)),
                     this,SLOT(slot_setTatgetChanged()));
    m_ui->radioButton_Base->setChecked(true);
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
    updatePoseBox(m_TorchPtr->Trans_O2M.getValue());
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
            if(objPtr->isDerivedFrom(Robot::MechanicRobot::getClassTypeId())){
                if(!static_cast<Robot::MechanicRobot*>(objPtr)->TorchAssembled()){
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

bool TaskBoxTorchToolSetupPanel::checkIfTorchInfoFullfilled()
{
    bool isValid = true;
    isValid &= !m_TorchPtr->Trans_O2M.getValue().isIdentity();
    isValid &= !m_TorchPtr->Trans_M2T.getValue().isIdentity();
    isValid &= !m_ui->lineEdit_brand->text().isEmpty();
    isValid &= m_ui->doubleSpinBox_tubeLength->value() != 0;
    return isValid;
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
    Q_EMIT Signal_updateDraggerPose(m_TorchPtr->getPose_ABSToolMount());
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
        displayPose = m_TorchPtr->getPose_ABSToolMount();
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
        Q_EMIT Signal_updateDraggerPose(m_TorchPtr->getPose_ABSToolMount());
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

void TaskBoxTorchToolSetupPanel::slot_finishSetupButtonClicked()
{
    if(creatingMode){
        if(!checkIfTorchInfoFullfilled()){
            QMessageBox::warning(this, tr("Warning"),tr("Torch Tool key information is missing, Please Check Panels!"));
            return;
        }
        m_TorchPtr->ToolBrand.setValue(m_ui->lineEdit_brand->text().toStdString().c_str());
        m_TorchPtr->m_Info.tube_Type = (Robot::Type_TorchTube)m_ui->comboBox_tubeType->currentIndex();
        m_TorchPtr->m_Info.tube_Length = m_ui->doubleSpinBox_tubeLength->value();
        m_TorchPtr->m_Info.torch_Name = m_TorchPtr->ToolBrand.getStrValue() + "_" +
                                        std::to_string(m_TorchPtr->m_Info.tube_Length) + "_" +
                                        Robot::StrList_TorchTubeType[(int)m_TorchPtr->m_Info.tube_Type];
        if(m_TorchPtr->saveTool()){
            QString msg = tr("Torch param file is saved as:") + tr(m_TorchPtr->FilePath_Param.getValue());
            QMessageBox::information(NULL, tr("Message"),msg);
        }
    }
//    this->close();
    Q_EMIT Signal_finishSetup();
}


#include "moc_TaskBoxTorchToolSetupPanel.cpp"
