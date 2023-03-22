
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <QMessageBox>

#include <App/Application.h>
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Document.h>

#include <Mod/Robot/Gui/ui_TaskBox_EdgeBasedTracEditor.h>
#include <Mod/Robot/Gui/ui_TaskBox_EdgeTracGenerator.h>

#include "TaskBox_EdgeBasedTracEditor.h"
//#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"
#include "Mod/Robot/App/Mechanics/MechanicGroup.h"

//using namespace RD_CAD_Utility;
using namespace RobotGui;
// EdgeTaskBox
TaskBox_EdgeBasedTracEditor::TaskBox_EdgeBasedTracEditor(Robot::RobotTracObject *t_TracObject, QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("Edge-Based Trac Editor"), true, parent) {
  if (t_TracObject == nullptr || !t_TracObject->isDerivedFrom(Robot::EdgebasedTracObject::getClassTypeId()))
    return;
  initEdgeTracEditor(t_TracObject);
  initUI_EditEdgePanel();
}


bool TaskBox_EdgeBasedTracEditor::initEdgeTracEditor(Robot::RobotTracObject *t_TracObjPtr)
{
    if(t_TracObjPtr == nullptr)
        return false;

    m_EdgeTracPtr = static_cast<Robot::EdgebasedTracObject*>(t_TracObjPtr);
    if(m_EdgeTracPtr == nullptr)
        return false;
    m_DocPtr = m_EdgeTracPtr->getDocument();
    m_PntAdjustments = std::vector<Base::Placement>(m_EdgeTracPtr->getWaypointData().size(),
                                                    Base::Placement());
    auto t_OperatorPtr = m_DocPtr->getObject(t_TracObjPtr->getOperatorName().c_str());
    if(t_OperatorPtr->isDerivedFrom(Robot::MechanicGroup::getClassTypeId())){
        m_GroupPtr = static_cast<Robot::MechanicGroup*>(t_OperatorPtr);
        if(m_GroupPtr == nullptr){
            Base::Console().Warning("TaskBox_EdgeTracEditor: Unable to Find valid GroupPtr to init Editor");
            return false;
        }
        m_EdgeTracPtr->CurrentExternalAxis.setValues(m_GroupPtr->getCurrentExtAxisValue());
        switch(m_EdgeTracPtr->getTracType()){
        case Robot::TracType::SCANTRAC:
            m_GroupPtr->setCurrentToolType(Robot::ToolType::_2DScanner);
            m_GroupPtr->setCurrentToolActive(true);
            break;
        case Robot::TracType::SEAMTRAC:
            m_GroupPtr->setCurrentToolType(Robot::ToolType::WeldTorch);
            break;
        }
        m_GroupPtr->Activated.setValue(true);
        return true;
    }
    else{
        std::string message = std::string("TaskBox_EdgeTracEditor: ") + std::string(t_OperatorPtr->getNameInDocument()) + std::string(" is Not Derived From MechanicGroup");
        Base::Console().Message(message.c_str());
    }
    return false;
}


void TaskBox_EdgeBasedTracEditor::initUI_EditEdgePanel()
{
    if(m_FaceSelection == nullptr){
        m_FaceSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Face COUNT 2",
                                                                    &(m_EdgeTracPtr->FaceSource));
        this->groupLayout()->addWidget(m_FaceSelection);
        m_FaceSelection->hide();
    }
    if(m_EdgeSelection == nullptr){
        m_EdgeSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Edge COUNT 1",
                                                                    &(m_EdgeTracPtr->EdgeSource));
        this->groupLayout()->addWidget(m_EdgeSelection);
        m_EdgeSelection->hide();
    }
    if(m_ui == nullptr){
        m_ui = new Ui_TaskBox_EdgeBasedTracEditor;
        m_proxy = new QWidget();
        m_ui->setupUi(m_proxy);
        this->groupLayout()->addWidget(m_proxy);
    }

    initUI_TracDefineTab();

}

void TaskBox_EdgeBasedTracEditor::initUI_TracDefineTab()
{
    initUI_Generationbox();
    initUI_PoseEditbox();
}

void TaskBox_EdgeBasedTracEditor::initUI_Generationbox()
{
    // Generation Panel
    QObject::connect(m_ui->radioButton_Gp1Robot, SIGNAL(clicked(bool)),
                     this, SLOT(slot_changeTargetRobotID()));
    QObject::connect(m_ui->radioButton_Gp2Robot, SIGNAL(clicked(bool)),
                     this, SLOT(slot_changeTargetRobotID()));
    if(m_GroupPtr!=nullptr){
        if(!m_GroupPtr->LinkedRobotName_1.getStrValue().empty())
            m_ui->radioButton_Gp1Robot->setEnabled(true);
        if(!m_GroupPtr->LinkedRobotName_2.getStrValue().empty())
            m_ui->radioButton_Gp2Robot->setEnabled(true);
        m_ui->radioButton_Gp1Robot->setChecked(true);
        m_RobotID = 1;
    }

    /// References
    m_ui->checkBox_flipTracDir->setChecked(m_EdgeTracPtr->Reverse_EdgeDir.getValue());
    m_ui->checkBox_RevFace1->setChecked(m_EdgeTracPtr->Reverse_RefFace_1.getValue());
    m_ui->checkBox_RevFace2->setChecked(m_EdgeTracPtr->Reverse_RefFace_2.getValue());
    QObject::connect(m_ui->checkBox_flipTracDir, SIGNAL(clicked(bool)),
                     this, SLOT(slot_changeEdgeDirection(bool)));
    QObject::connect(m_ui->checkBox_RevFace1, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipReference_1(bool)));
    QObject::connect(m_ui->checkBox_RevFace2, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipReference_2(bool)));

    /// Trac Type
    switch(m_EdgeTracPtr->getTracType()){
    case Robot::TracType::SEAMTRAC:
        m_ui->tabWidget->setTabEnabled(1,false);
        break;
    case Robot::TracType::SCANTRAC:
        m_ui->tabWidget->setTabEnabled(2,false);
        break;
    default:
        break;
    }

    /// Stp Length
    m_ui->doubleSpinBox_InterpoDis->setValue(m_EdgeTracPtr->InterpoDist.getValue());
    QObject::connect(m_ui->doubleSpinBox_InterpoDis, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeEdgeInterpoLen(double)));
    QObject::connect(m_ui->pushButton_generateConstraint, SIGNAL(clicked()),
                     this, SLOT(slot_generateConstraint()));

}

void TaskBox_EdgeBasedTracEditor::initUI_PoseEditbox()
{

    m_ui->groupBox_PoseEditor->setEnabled(!m_EdgeTracPtr->getWaypointData().empty());

    /// Visualizer
    QObject::connect(m_ui->radioButton_RobotViz,SIGNAL(clicked(bool)),
                     this, SLOT(slot_visualizerChanged()));
    QObject::connect(m_ui->radioButton_MarkerViz,SIGNAL(clicked(bool)),
                     this, SLOT(slot_visualizerChanged()));
    slot_visualizerChanged();

    /// Slider
    m_ui->horizontalSlider_EdgePntPos->setEnabled(true);
    m_ui->horizontalSlider_EdgePntPos->setMaximum(m_EdgeTracPtr->getWaypointData().size());
    m_ui->horizontalSlider_EdgePntPos->setTickPosition(QSlider::TickPosition::TicksBelow);
    QObject::connect(m_ui->horizontalSlider_EdgePntPos, SIGNAL(valueChanged(int)),
                     this, SLOT(slot_sliderPositionChanged(int)));
    m_ui->horizontalSlider_EdgePntPos->setSliderPosition(1);
    QObject::connect(m_ui->pushButton_CutHere, SIGNAL(clicked()),
                     this, SLOT(slot_cutRestEdgeFromCurrentPoint()));
    QObject::connect(m_ui->pushButton_SaveRest, SIGNAL(clicked()),
                     this, SLOT(slot_cutFormerEdgeFromCurrentPoint()));

    /// Pose Adjustment
    QObject::connect(m_ui->doubleSpinBox_PoseYaw, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));
    QObject::connect(m_ui->doubleSpinBox_PosePitch, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));
    QObject::connect(m_ui->doubleSpinBox_PoseRoll, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));
    QObject::connect(m_ui->doubleSpinBox_OffsetZ, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));
    QObject::connect(m_ui->doubleSpinBox_OffsetW, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));
    QObject::connect(m_ui->doubleSpinBox_OffsetB, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePoseAdjustment()));

    QObject::connect(m_ui->doubleSpinBox_WeldSpeed, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_setTracWeldSpeed(double)));
    QObject::connect(m_ui->doubleSpinBox_ScanSpeed, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_setTracScanSpeed(double)));
    QObject::connect(m_ui->pushButton_applyAdjustment, SIGNAL(clicked(bool)),
                     this, SLOT(slot_applyPoseAdjustmentToRest()));
    QObject::connect(m_ui->pushButton_FinishEdit, SIGNAL(clicked()),
                     this, SLOT(slot_finishEdit()));
}

TaskBox_EdgeBasedTracEditor::~TaskBox_EdgeBasedTracEditor() {}


void TaskBox_EdgeBasedTracEditor::slot_finishEdit() {
  if (m_EdgeTracPtr->getWaypointData().empty()) {
      Base::Console().Warning("TaskBox_EdgeTracEditor: No Valid Edge Trac Calculated, Removing Target Trac from Doc!\n");
      m_DocPtr->removeObject(m_EdgeTracPtr->getNameInDocument());
      Q_EMIT Signal_TracObjectEditDone(nullptr);
  }
  else{
      Base::Console().Message("TaskBox_EdgeTracEditor: Edge Trac Calculate Success!\n");
      Q_EMIT Signal_TracObjectEditDone(m_EdgeTracPtr);
  }
  enableRobotVisualizer(false, false);
  enableMarkerVisualizer(false);
  this->hide();
}

void TaskBox_EdgeBasedTracEditor::slot_generateConstraint()
{
    // Update References
    bool success = true;
    m_FaceSelection->sendSelection2Property();
    m_EdgeSelection->sendSelection2Property();
    success &= m_EdgeTracPtr->generateConstraint(m_RobotID);
    if(!success){
        Base::Console().Warning("TaskBox_EdgeBasedTracEditor: Failed To Generate Constraint from Selected Feature!\n");
        return;
    }
    m_EdgeTracPtr->setTracSafePoint(m_GroupPtr->getCurrentGroupPose(Robot::CordType::WCS));
    success &= m_EdgeTracPtr->generateProgram(m_RobotID);
    if(success){
        m_PntAdjustments = std::vector<Base::Placement>(m_EdgeTracPtr->getWaypointData().size(), Base::Placement());
        Q_EMIT Signal_updateTracObject(m_EdgeTracPtr->getRobotProgramSptr());
    }
    else{
        Base::Console().Warning("TaskBox_EdgeBasedTracEditor: Failed to Generate Program based on Constraint!\n");
    }
    initUI_PoseEditbox();
    return;
}


void TaskBox_EdgeBasedTracEditor::slot_updatePoseAdjustment()
{
    // Get New Rotations
    Base::Rotation t_Rot;
    double rotation_Yaw = m_ui->doubleSpinBox_PoseYaw->value();
    double rotation_Roll = m_ui->doubleSpinBox_PoseRoll->value();
    double rotation_Pit = m_ui->doubleSpinBox_PosePitch->value();
    t_Rot.setYawPitchRoll(rotation_Yaw,
                          rotation_Pit,
                          rotation_Roll);
    double tx = 0, ty = 0, tz = 0;
    tx = m_ui->doubleSpinBox_OffsetW->value();
    ty = m_ui->doubleSpinBox_OffsetB->value();
    tz = m_ui->doubleSpinBox_OffsetZ->value();

    m_PntAdjustments[c_PntID] = Base::Placement(Base::Vector3d(tx, ty, tz),
                                                t_Rot);
//    c_PntAdjustment.setPosition(Base::Vector3d(tx, ty, tz));
//    c_PntAdjustment.setRotation(t_Rot);

    updateVisualizer();
}

void TaskBox_EdgeBasedTracEditor::slot_setTracWeldSpeed(double n_speed)
{
    m_EdgeTracPtr->WeldSpeed.setValue(n_speed);
}

void TaskBox_EdgeBasedTracEditor::slot_changeEdgeDirection(bool flag)
{
    m_EdgeTracPtr->Reverse_EdgeDir.setValue(flag);
}


void TaskBox_EdgeBasedTracEditor::slot_flipReference_1(bool flag) {
    m_EdgeTracPtr->Reverse_RefFace_1.setValue(flag);
}

void TaskBox_EdgeBasedTracEditor::slot_flipReference_2(bool flag) {
    m_EdgeTracPtr->Reverse_RefFace_2.setValue(flag);
}

void TaskBox_EdgeBasedTracEditor::slot_changeEdgeInterpoLen(double newStp) {
    m_EdgeTracPtr->InterpoDist.setValue(newStp);
}

void TaskBox_EdgeBasedTracEditor::slot_sliderPositionChanged(int position) {
    c_PntID = position - 1;
    if(position - 1<0)
        c_PntID = 0;
    // Change ID
    m_ui->label_CurrentPointID->setText(QString::number(c_PntID));
    updateCurrentPntPose();
    updateVisualizer();
}

void TaskBox_EdgeBasedTracEditor::slot_cutRestEdgeFromCurrentPoint() {
  m_EdgeTracPtr->cutEdgeAtPoint(c_PntID, true);
  m_ui->horizontalSlider_EdgePntPos->setMaximum(m_EdgeTracPtr->getWaypointData().size());
}

void TaskBox_EdgeBasedTracEditor::slot_cutFormerEdgeFromCurrentPoint() {
  m_EdgeTracPtr->cutEdgeAtPoint(c_PntID, false);
  m_ui->horizontalSlider_EdgePntPos->setMaximum(m_EdgeTracPtr->getWaypointData().size());
}

void TaskBox_EdgeBasedTracEditor::changeEvent(QEvent *e) {
  if (e->type() == QEvent::LanguageChange) {
      m_ui->retranslateUi(this);
  }
  QWidget::changeEvent(e);
}

void TaskBox_EdgeBasedTracEditor::updatePoseAdjustmentPanel(const Base::Placement& ajst_Pose) {

  double t_x, t_y, t_z;
  double rx, ry, rz;
  t_x = ajst_Pose.getPosition().x;
  t_y = ajst_Pose.getPosition().y;
  t_z = ajst_Pose.getPosition().z;
  ajst_Pose.getRotation().getYawPitchRoll(rz, ry, rx);

  blockPoseEditboxSignal(true);
  m_ui->doubleSpinBox_OffsetZ->setValue(t_z);
  m_ui->doubleSpinBox_OffsetW->setValue(t_x);
  m_ui->doubleSpinBox_OffsetB->setValue(t_y);
  m_ui->doubleSpinBox_PoseRoll->setValue(rx);
  m_ui->doubleSpinBox_PosePitch->setValue(ry);
  m_ui->doubleSpinBox_PoseYaw->setValue(rz);
  blockPoseEditboxSignal(false);
}

void TaskBox_EdgeBasedTracEditor::setupDragger(bool enable) {
  if(m_EdgeTracPtr == nullptr)
      return;
  auto edge_VP = Gui::Application::Instance->activeDocument()->getViewProvider(m_EdgeTracPtr);
  if (enable) {
      if(m_MarkerViz == nullptr){
          m_MarkerViz = new RobotGui::InteractiveDragger(edge_VP->getRoot(),
                                                         m_PntOriginPose*m_PntAdjustments[c_PntID],
                                                         RobotGui::DraggerUsage::Display,
                                                         DraggerType::SpotLightDragger);
          m_MarkerViz->setAttachingViewProvider(edge_VP);
//          m_MarkerViz->enableSceneGraphSelection(false);
      }
  }
  else {
    if (m_MarkerViz){
//        m_MarkerViz->enableSceneGraphSelection(true);
        m_MarkerViz->destroyDragger();
        m_MarkerViz = nullptr;
    }
  }
}

void TaskBox_EdgeBasedTracEditor::updateVisualizer()
{
    auto t_Pose = m_PntOriginPose * m_PntAdjustments[c_PntID];
    updateDraggerPose(t_Pose);
    updateRobotPose(t_Pose);
}

void TaskBox_EdgeBasedTracEditor::updateDraggerPose(const Base::Placement &new_Pose) {
  if (!flag_VizByMarker || m_MarkerViz == nullptr)
    return;
  m_MarkerViz->setDraggerPosition(new_Pose);
}

void TaskBox_EdgeBasedTracEditor::enableRobotVisualizer(bool enable, bool restorePose) {
  if (m_EdgeTracPtr == nullptr || m_GroupPtr == nullptr)
    return;
  if(enable){
      m_InitPoses = m_GroupPtr->getCurrentGroupPose(Robot::CordType::ACS);
      flag_VizByRobot = true;
  }
  else{
      if(restorePose)
        m_GroupPtr->setGroupPose(m_InitPoses);
      flag_VizByRobot = false;
  }
}

void TaskBox_EdgeBasedTracEditor::enableMarkerVisualizer(bool enable)
{
    flag_VizByMarker = enable;
    setupDragger(flag_VizByMarker);
}

void TaskBox_EdgeBasedTracEditor::updateCurrentPntPose()
{
    switch(m_RobotID){
    case 1:
        m_PntOriginPose = m_EdgeTracPtr->getRobotProgramSptr()->getWaypoint_byPosition(c_PntID)->getWPCartPose_GP1();
        break;
    case 2:
        m_PntOriginPose = m_EdgeTracPtr->getRobotProgramSptr()->getWaypoint_byPosition(c_PntID)->getWPCartPose_GP2();
        break;
    }
}

void TaskBox_EdgeBasedTracEditor::slot_setTracScanSpeed(double n_speed)
{
    m_EdgeTracPtr->ScanSpeed.setValue(n_speed);
}

void TaskBox_EdgeBasedTracEditor::slot_applyPoseAdjustmentToRest()
{
    m_EdgeTracPtr->setAdjustPoseToRest(c_PntID, m_RobotID, m_PntAdjustments[c_PntID]);

    m_PntAdjustments = std::vector<Base::Placement>(m_EdgeTracPtr->getWaypointData().size(), Base::Placement());
    updatePoseAdjustmentPanel(Base::Placement());
    Q_EMIT Signal_updateTracObject(m_EdgeTracPtr->getRobotProgramSptr());
}

void TaskBox_EdgeBasedTracEditor::slot_updateDiffByCurrentTipPose(bool checked)
{
    auto c_TipPose = m_GroupPtr->getGroupTipPose();
    auto diff_Rot = m_PntOriginPose.getRotation().inverse() * c_TipPose.getRotation();
    auto diff_Trans = c_TipPose.getPosition() - m_PntOriginPose.getPosition();
    updatePoseAdjustmentPanel(Base::Placement(diff_Trans,diff_Rot));
    slot_updatePoseAdjustment();
}

void TaskBox_EdgeBasedTracEditor::slot_changeTargetRobotID()
{
    if(m_ui->radioButton_Gp1Robot->isChecked())
        m_RobotID = 1;
    else if(m_ui->radioButton_Gp2Robot->isChecked())
        m_RobotID = 2;
}


void TaskBox_EdgeBasedTracEditor::slot_visualizerChanged()
{
    enableRobotVisualizer(m_ui->radioButton_RobotViz->isChecked());
    enableMarkerVisualizer(m_ui->radioButton_MarkerViz->isChecked());
}


void TaskBox_EdgeBasedTracEditor::updateRobotPose(const Base::Placement& new_Pose) {
  if(!flag_VizByRobot || m_GroupPtr == nullptr)
      return;
  m_GroupPtr->setTipPose(new_Pose);
}

void TaskBox_EdgeBasedTracEditor::blockPoseEditboxSignal(bool block)
{
    m_ui->doubleSpinBox_OffsetZ->blockSignals(block);
    m_ui->doubleSpinBox_OffsetW->blockSignals(block);
    m_ui->doubleSpinBox_OffsetB->blockSignals(block);

    m_ui->doubleSpinBox_PoseYaw->blockSignals(block);
    m_ui->doubleSpinBox_PoseRoll->blockSignals(block);
    m_ui->doubleSpinBox_PosePitch->blockSignals(block);
}

#include "moc_TaskBox_EdgeBasedTracEditor.cpp"
