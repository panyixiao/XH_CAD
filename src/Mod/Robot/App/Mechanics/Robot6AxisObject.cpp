// Created By Yixiao 2022-04-16

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "Robot6AxisObject.h"
#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::Robot6AxisObject, App::GeoFeature)


Robot6AxisObject::Robot6AxisObject()
:flag_updateJntVals(false)
{
    ADD_PROPERTY_TYPE(UseTracIK,(true),"Property",Prop_None,"Select IK Solver");
    ADD_PROPERTY_TYPE(Activated,(false),"Property",Prop_None,"Is robot activated");
    ADD_PROPERTY_TYPE(Visible,(true),"Property",Prop_None,"Is Robot Visiable");

    ADD_PROPERTY_TYPE(FilePath_URDF,(""),"Files",Prop_None,"Included file with robot definition about robot model");

    ADD_PROPERTY_TYPE(ExternalAxisName,(""),"Property",Prop_None,"The Name of Assembled External Axis");
    ADD_PROPERTY_TYPE(Pose_Reference,(Base::Placement()),"Property",Prop_None,"Pose of Reference Origin");
    ADD_PROPERTY_TYPE(Pose_Ref2Base,(Base::Placement()),"Property",Prop_None,"Translation From Reference To Base");
    ADD_PROPERTY_TYPE(Pose_Flan,(Base::Placement()),"Property",Prop_None,"Tcp Pose in World Coord");

    ADD_PROPERTY_TYPE(HomePose ,(0.0), "Robot kinematic",Prop_None,"Axis position for home");
    HomePose.setSize(6);
    ADD_PROPERTY_TYPE(MainAxisValues, (0.0), "Robot kinematic", Prop_None, "Axis angle of Standard Robot Axis Value");
    MainAxisValues.setSize(6);
    ADD_PROPERTY_TYPE(ExtrAxisValues,(0.0), "Robot kinematic", Prop_None, "Axis angle of Extra Axis Value (J7/J8)");
    ExtrAxisValues.setSize(2);
    ADD_PROPERTY_TYPE(AxisRatedSpeed, (0.0), "Robot kinematic", Prop_None, "Max Moving Speed of each Axis");

    ADD_PROPERTY_TYPE(AxisSpeedRatio, (1.0), "Robot kinematic", Prop_None, "Speed Limits for each");

    ADD_PROPERTY_TYPE(UpperLimits_Real, (0.0), "Robot kinematic", Prop_None, "Real Upper Limits");
    UpperLimits_Real.setSize(6);
    ADD_PROPERTY_TYPE(UpperLimits_Soft, (0.0), "Robot kinematic", Prop_None, "Soft Upper Limits");
    UpperLimits_Soft.setSize(6);

    ADD_PROPERTY_TYPE(LowerLimits_Real, (0.0), "Robot kinematic", Prop_None, "Real Lower Limits");
    LowerLimits_Real.setSize(6);
    ADD_PROPERTY_TYPE(LowerLimits_Soft, (0.0), "Robot kinematic", Prop_None, "Soft Lower Limits");
    LowerLimits_Soft.setSize(6);

    ADD_PROPERTY(Wrist_NonFlip,(true));
    ADD_PROPERTY(ForeArm_onRight,(true));
    ADD_PROPERTY(Elbow_Upward,(true));
    ADD_PROPERTY(EnableArmConfiguration,(false));


    ADD_PROPERTY_TYPE(isDriven, (false),"Property", Prop_Hidden,"");
    // Tool
    ADD_PROPERTY_TYPE(CurrentToolIndex,(0),"Tool Setup", Prop_None,"Current ID of Selected Tool");
    ADD_PROPERTY_TYPE(TorchIndex,(1),"Tool Setup", Prop_Hidden,"ToolID of Torch Tool");
    ADD_PROPERTY_TYPE(ScannerIndex,(2),"Tool Setup", Prop_Hidden,"ToolID of Torch Tool");
    ADD_PROPERTY_TYPE(CameraIndex,(3),"Tool Setup", Prop_Hidden,"ToolID of Torch Tool");
    ADD_PROPERTY_TYPE(TorchShape,(0),"Tool Setup",Prop_None,"Link to the Shape is used as Torch");
    ADD_PROPERTY_TYPE(Trans_Flan2TorchTip,(Base::Placement()),"Tool Setup",Prop_None,"Translation from Flan to Torch Tip");
    ADD_PROPERTY_TYPE(SensorShape,(0),"Tool Setup",Prop_None,"Link to the Shape is used as Sensor");
    ADD_PROPERTY_TYPE(Trans_Flan2SensorOrigin,(Base::Placement()),"Tool Setup",Prop_None,"Translation from Flan to SensorOrigin");

    ADD_PROPERTY_TYPE(TeachCoordIndex,(0),"",Prop_None, "Current Teach Coord Index");

    ADD_PROPERTY_TYPE(LinkedEdgeFeature, (0), "", Prop_None, "Linked Edge Feature for attachment and tip setup");
    LinkedEdgeFeature.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY_TYPE(LinkedFaceFeature, (0), "", Prop_None, "Linked Face Feature for attachment and tip setup");
    LinkedFaceFeature.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(MoveWithAttachedBody,(false));
    MoveWithAttachedBody.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(InteractiveTeach,(false));
    InteractiveTeach.setStatus(App::Property::Status::Hidden, true);

    m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
    m_kinematicModel.setMechanicType(MechanicType::M_Robot);
    m_kinematicModel.updateKinematicModelByConfig(ConfigType::NON);
}

Robot6AxisObject::~Robot6AxisObject()
{
}

DocumentObjectExecReturn *Robot6AxisObject::execute()
{
    return StdReturn;
}

DocumentObjectExecReturn *Robot6AxisObject::recompute()
{
    m_kinematicModel.updateJointFrames();
    return this->execute();
}

short Robot6AxisObject::mustExecute(void) const
{
    return 0;
}

PyObject *Robot6AxisObject::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void Robot6AxisObject::onChanged(const Property* prop)
{
   if(prop == &FilePath_URDF){
        auto result = m_kinematicModel.readURDFfiles(FilePath_URDF.getValue());
        if(!result){
            // Export Warning
        }
        else{
            AxisRatedSpeed.setValues(m_kinematicModel.getJointSpeedLimits());
            UpperLimits_Real.setValues(m_kinematicModel.getJointMaxAngles());
            UpperLimits_Soft.setValues(m_kinematicModel.getJointMaxAngles());
            LowerLimits_Real.setValues(m_kinematicModel.getJointMinAngles());
            LowerLimits_Soft.setValues(m_kinematicModel.getJointMinAngles());
        }
   }
   else if(prop == &UpperLimits_Soft){
        m_kinematicModel.setJointMaxLimits(UpperLimits_Soft.getValues());
   }
   else if(prop == &LowerLimits_Soft){
        m_kinematicModel.setJointMinLimits(LowerLimits_Soft.getValues());
   }
   else if(prop == &MainAxisValues){
        // Change RobotState By set Joint Values
        if(!flag_updateTcpPose){
            flag_updateJntVals = true;
            // Update RobotModel
            m_kinematicModel.setJointAngles(MainAxisValues.getValues());
            Pose_Flan.setValue(getCurrentFlanPose(CoordOrigin::World));
            udpateToolPosition();
            flag_updateJntVals = false;
        }
        // Change RobotState By set TCP pose
        else{
            m_kinematicModel.updateJointFrames();
        }
   }
   else if(prop == &Pose_Flan && !flag_updateJntVals){
        // Update KinematicModel JntVals
        if(m_kinematicModel.setTo(Base::Placement(Pose_Flan.getValue()))){
            // Update JntVals to AxisValues
            flag_updateTcpPose = true;
            updateAxisValues();
            udpateToolPosition();
            flag_updateTcpPose = false;
        }
   }
   else if(prop == &UseTracIK){
        if(UseTracIK.getValue())
            m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
        else
            m_kinematicModel.setIKsolverType(IK_SolverType::KDL_IK);
   }
   else if(prop == &Elbow_Upward ||
           prop == &EnableArmConfiguration){
       updateRobotConfiguration();
   }
   App::GeoFeature::onChanged(prop);
}

void Robot6AxisObject::onDocumentRestored()
{
    if(!TorchName.isEmpty() && m_Torch == nullptr){
        installTool(TorchName.getValue());
    }
    if(!ScannerName.isEmpty() && m_Scanner == nullptr){
        installTool(ScannerName.getValue());
    }

    MainAxisValues.setValues(MainAxisValues.getValues());

    App::GeoFeature::onDocumentRestored();
}

void Robot6AxisObject::updateRobotConfiguration()
{
    ConfigType t_Type;

    if(Wrist_NonFlip.getValue()){
        if(ForeArm_onRight.getValue()){
            if(Elbow_Upward.getValue())
                t_Type = ConfigType::NRU;
            else
                t_Type = ConfigType::NRD;
        }
        else{
            if(Elbow_Upward.getValue())
                t_Type = ConfigType::NLU;
            else
                t_Type = ConfigType::NLD;
        }
    }else{
        if(ForeArm_onRight.getValue()){
            if(Elbow_Upward.getValue())
                t_Type = ConfigType::FRU;
            else
                t_Type = ConfigType::FRD;
        }
        else{
            if(Elbow_Upward.getValue())
                t_Type = ConfigType::FLU;
            else
                t_Type = ConfigType::FLD;
        }
    }
    if(EnableArmConfiguration.getValue())
       m_kinematicModel.updateKinematicModelByConfig(t_Type);
    else
       m_kinematicModel.updateKinematicModelByConfig(ConfigType::NON);
}

bool Robot6AxisObject::setRobotTipPose(const Base::Placement &n_TipPose,
                                       CoordOrigin pose_Coord,
                                       Base::Placement coordOrigin_Pose)
{
    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getCurrentBasePose().inverse();
    Base::Placement targetPose;
    switch(pose_Coord){
        case CoordOrigin::World:
        targetPose = base_inv.toMatrix() *
                     n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
            break;
        case CoordOrigin::Robot:
        targetPose = n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
            break;

        case CoordOrigin::Flan:
        targetPose = n_TipPose;
            break;

        case CoordOrigin::Object:{
        targetPose = base_inv.toMatrix() *
                     coordOrigin_Pose.toMatrix() *
                     n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
        }
            break;
    }

    if(m_kinematicModel.setTo(targetPose)){
        Pose_Flan.setValue(targetPose);
        return true;
    }

    return false;
}

const Base::Placement Robot6AxisObject::getCurrentTipPose(CoordOrigin ref_Origin,
                                                          Base::Placement origin_Pose) const
{
    Base::Placement result, tool_transform;
    tool_transform = getToolTipTranslation();

    result = Base::Placement(getCurrentFlanPose(ref_Origin,
                                                origin_Pose).toMatrix() *
                             tool_transform.toMatrix());

    return result;
}

const Base::Placement Robot6AxisObject::getCurrentFlanPose(const CoordOrigin &ref_Origin,
                                                           Base::Placement origin_Pose) const
{
    Base::Placement c_flanPose;
    if(!m_kinematicModel.initialized())
        return c_flanPose;

    Base::Placement abs_flanPose = m_kinematicModel.getTcp();

    switch(ref_Origin){
    case CoordOrigin::World:
        c_flanPose= Base::Placement(getCurrentBasePose().toMatrix() *
                                    abs_flanPose.toMatrix());
        break;
    case CoordOrigin::Robot:
        c_flanPose = abs_flanPose;
        break;
    case CoordOrigin::Flan:
        c_flanPose = Base::Placement();
        break;
    case CoordOrigin::Object:{
//        auto originPose_inv = origin_Pose;
//        originPose_inv.inverse();
//        c_flanPose= Base::Placement(originPose_inv.toMatrix() * getCurrentFlanPose(CoordOrigin::World).toMatrix() );
        break;
    }
    default:
        break;
    }
    return c_flanPose;
}


void Robot6AxisObject::setTipPoseByDragger(const Base::Placement &t_DraggerPose)
{
    Base::Placement n_TipPose = t_DraggerPose;

    switch(m_TeachCoord){
    case TeachCoord::World:{
        n_TipPose.setRotation(getCurrentTipPose(CoordOrigin::World).getRotation());
    }
        break;
    case TeachCoord::RobotBase:{
        n_TipPose.setRotation(getCurrentTipPose(CoordOrigin::Robot).getRotation());
    }
        break;
    case TeachCoord::Tip:
        break;
    }

    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getCurrentBasePose().inverse();

    auto tipPose_RbtFrame = base_inv * n_TipPose;
    Pose_Flan.setValue(Base::Placement(tipPose_RbtFrame.toMatrix()*
                                        toolTrans.inverse().toMatrix()));
}

void Robot6AxisObject::setTipPoseByDiff(const Base::Placement &movement)
{
    Base::Placement c_TipPose;

    Base::Placement toolTrans = getToolTipTranslation();
    Base::Placement trans_stp = Base::Placement(movement.getPosition(), Base::Rotation());
    Base::Placement rotat_stp = Base::Placement(Base::Vector3d(), movement.getRotation());

    switch(m_TeachCoord){
    case TeachCoord::World:{
        c_TipPose = getCurrentTipPose(CoordOrigin::World);
        Base::Placement trans_org = Base::Placement(c_TipPose.getPosition(), Base::Rotation());
        Base::Placement rotat_org = Base::Placement(Base::Vector3d(), c_TipPose.getRotation());
        Pose_Flan.setValue(Base::Placement(getCurrentBasePose().inverse().toMatrix() *
                                            trans_stp.toMatrix()*
                                            trans_org.toMatrix()*
                                            rotat_stp.toMatrix()*
                                            rotat_org.toMatrix()*
                                            toolTrans.inverse().toMatrix()));
    }
        break;
    case TeachCoord::RobotBase:{
        c_TipPose = getCurrentTipPose(CoordOrigin::Robot);
        Base::Placement trans_org = Base::Placement(c_TipPose.getPosition(), Base::Rotation());
        Base::Placement rotat_org = Base::Placement(Base::Vector3d(), c_TipPose.getRotation());
        Pose_Flan.setValue(Base::Placement(trans_stp.toMatrix()*
                                            trans_org.toMatrix()*
                                            rotat_stp.toMatrix()*
                                            rotat_org.toMatrix()*
                                            toolTrans.inverse().toMatrix()));
    }
        break;
    case TeachCoord::Tip:
        c_TipPose = getCurrentTipPose(CoordOrigin::World);
        Base::Placement trans_org = Base::Placement(c_TipPose.getPosition(), Base::Rotation());
        Base::Placement rotat_org = Base::Placement(Base::Vector3d(), c_TipPose.getRotation());
        Pose_Flan.setValue(Base::Placement(getCurrentBasePose().inverse().toMatrix() *
                                            trans_org.toMatrix()*
                                            rotat_org.toMatrix()*
                                            trans_stp.toMatrix()*
                                            rotat_stp.toMatrix()*
                                            toolTrans.inverse().toMatrix()));
        break;
    }
}

const Base::Placement Robot6AxisObject::getToolTipTranslation() const
{
    Base::Placement toolTrans;
    if(CurrentToolIndex.getValue() == 0){
        toolTrans = Base::Placement();
    }
    else if(CurrentToolIndex.getValue() == TorchIndex.getValue()){
        toolTrans = m_Torch != nullptr?m_Torch->getTransform_Mount2Front():Base::Placement();
    }
    else if(CurrentToolIndex.getValue() == ScannerIndex.getValue()){
        toolTrans = m_Scanner != nullptr?m_Scanner->getTransform_Mount2Front():Base::Placement();
    }

    return toolTrans;
}

const Base::Placement Robot6AxisObject::getToolFrameTrans(const uint ToolID) const
{
    // TODO: Re-factor robot tool frame management mechanism
    if(ToolID == TorchIndex.getValue() && m_Torch!=nullptr)
        return m_Torch->getTransform_Mount2Front();
    if(ToolID == ScannerIndex.getValue() && m_Scanner!=nullptr)
        return m_Scanner->getTransform_Mount2Front();
    return Base::Placement();
}


void Robot6AxisObject::installTool(const char *tool_Name)
{
    if(tool_Name == nullptr)
        return;
    auto t_Tool = static_cast<Robot::ToolObject*>(getDocument()->getObject(tool_Name));
    if(t_Tool == nullptr)
        return;
    switch(t_Tool->getToolType()){
    case ToolType::WeldTorch:
        m_Torch = static_cast<Robot::TorchObject*>(t_Tool);
        TorchName.setValue(t_Tool->getNameInDocument());
        CurrentToolIndex.setValue(TorchIndex.getValue());
        break;
    case ToolType::Scanner:
        m_Scanner = static_cast<Robot::ScannerObject*>(t_Tool);
        ScannerName.setValue(t_Tool->getNameInDocument());
        CurrentToolIndex.setValue(ScannerIndex.getValue());
        break;
    default:
        break;
    }
    udpateToolPosition();
}

void Robot6AxisObject::uninstallTool(ToolObject *t_Tool)
{
    if(t_Tool == nullptr)
        return;
    switch(t_Tool->getToolType()){
    case ToolType::WeldTorch:
        m_Torch = nullptr;
        TorchName.setValue("");
        break;
    case ToolType::Scanner:
        m_Scanner = nullptr;
        ScannerName.setValue("");
        break;
    default:
        break;
    }
}

void Robot6AxisObject::udpateToolPosition()
{
    if(m_Scanner){
        m_Scanner->updateToolMountPose(getCurrentFlanPose(CoordOrigin::World));
    }
    if(m_Torch){
        m_Torch->updateToolMountPose(getCurrentFlanPose(CoordOrigin::World));
    }
}

void Robot6AxisObject::setCurrentToolType(const ToolType &t_Type)
{
    switch(t_Type){
    case ToolType::NoTool:
        CurrentToolIndex.setValue(0);
        break;
    case ToolType::WeldTorch:
        CurrentToolIndex.setValue(TorchIndex.getValue());
        break;
    case ToolType::Scanner:
        CurrentToolIndex.setValue(ScannerIndex.getValue());
        break;
    default:
        CurrentToolIndex.setValue(0);
        break;
    }
}

void Robot6AxisObject::setCurrentToolActive(bool activated)
{
    switch(getCurrentTool()){
    case ToolType::NoTool:
        break;
    case ToolType::WeldTorch:
        if(m_Torch){
            m_Torch->SparkOn.setValue(activated);
        }
        break;
    case ToolType::Scanner:
        if(m_Scanner){
            m_Scanner->LaserOn.setValue(activated);
        }
        break;
    }
}

const ToolType Robot6AxisObject::getCurrentTool() const
{
    ToolType c_Type;
    if(CurrentToolIndex.getValue() == TorchIndex.getValue())
        c_Type = ToolType::WeldTorch;
    else if(CurrentToolIndex.getValue() == ScannerIndex.getValue())
        c_Type = ToolType::Scanner;
    else if(CurrentToolIndex.getValue() == CameraIndex.getValue())
        c_Type = ToolType::DepthCamera;
    else
        c_Type = ToolType::NoTool;
    return c_Type;
}

std::vector<DocumentObject *> Robot6AxisObject::getChildrenList() const
{
    std::vector<DocumentObject *> result;
    if(m_Scanner)
        result.push_back(m_Scanner);
    if(m_Torch)
        result.push_back(m_Torch);
    return result;
}

const Base::Placement Robot6AxisObject::getTeachDraggerPose() const
{
    auto result = getCurrentTipPose(CoordOrigin::World);
    Base::Rotation t_Rot;
    switch (m_TeachCoord) {
    case TeachCoord::World:
        t_Rot = Base::Rotation();
        break;
    case  TeachCoord::RobotBase:
        t_Rot = getCurrentBasePose().getRotation();
        break;
    case  TeachCoord::Tip:
        t_Rot = result.getRotation();
    default:
        break;
    }
    result.setRotation(t_Rot);
    return result;
}

const Base::Placement Robot6AxisObject::getCurrentBasePose() const
{
    return Base::Placement(Pose_Reference.getValue() * Pose_Ref2Base.getValue());
}

const Base::Placement Robot6AxisObject::getSelectedFeatureCenter() const
{
    // TODO: Consider Edge Center as well
    return CAD_Utility::calculateLinkedFaceCenter(LinkedFaceFeature);
}

bool Robot6AxisObject::setCurrentPoseAsHome()
{
    auto c_jntVals = m_kinematicModel.getJointAngles();
    if(c_jntVals.size() != HomePose.getValues().size())
        return false;
    HomePose.setValues(c_jntVals);
    return true;
}

bool Robot6AxisObject::restoreHomePose()
{
    if(m_kinematicModel.setJointAngles(HomePose.getValues())){
        updateAxisValues();
        return true;
    }
    return false;
}

const float Robot6AxisObject::getJointMaxAngle(const int t_ID) const
{
    if(t_ID<6)
        return m_kinematicModel.getMaxAngle(t_ID);
    else
        return 0;
}

const float Robot6AxisObject::getJointMinAngle(const int t_ID) const
{
    if(t_ID<6)
        return m_kinematicModel.getMinAngle(t_ID);
    else
        return 0;
}

const float Robot6AxisObject::getJointAngle(const int t_ID) const
{
    if(t_ID < 6)
        return m_kinematicModel.getJointAngle(t_ID);
    else
        return 0;
}

const std::vector<double> Robot6AxisObject::getJointMaxAngles() const
{
    return m_kinematicModel.getJointMaxAngles();
}

const std::vector<double> Robot6AxisObject::getJointMinAngles() const
{
    return m_kinematicModel.getJointMinAngles();
}


void Robot6AxisObject::restoreJointLimits()
{
    m_kinematicModel.setJointMaxLimits(UpperLimits_Real.getValues());
    m_kinematicModel.setJointMinLimits(LowerLimits_Real.getValues());
}

bool Robot6AxisObject::JointDirInverted(uint j_id)
{
    return m_kinematicModel.JointDirInverted(j_id);
}

void Robot6AxisObject::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    m_kinematicModel.Save(writer);
}

void Robot6AxisObject::Restore(Base::XMLReader &reader)
{
    flag_updateJntVals = true;
    App::GeoFeature::Restore(reader);
    m_kinematicModel.Restore(reader);
    flag_updateJntVals = false;
}

KinematicModel &Robot6AxisObject::getKinematicModelRef()
{
    return m_kinematicModel;
}

bool Robot6AxisObject::flipAxisDirection(uint jntID, bool invert)
{
    return m_kinematicModel.flipJointDir(jntID, invert);
}


bool Robot6AxisObject::setAxisValues(const std::vector<double> t_Vals)
{
    if(t_Vals.size()!=getJointNumbers())
        return false;
    std::vector<double> t_angles;
    for(int i = 0; i<6; i++){
        t_angles.push_back(t_Vals[i]);
    }
    MainAxisValues.setValues(t_angles);
    ExtrAxisValues.setValues(std::vector<double>(t_Vals.begin()+6, t_Vals.end()));
    return true;
}

bool Robot6AxisObject::setJointAngle(int jntID, float jntAngle)
{
    if(0<= jntID && jntID < MainAxisValues.getValues().size()){
        m_kinematicModel.setJointAngle(jntID, jntAngle);
        updateAxisValues();
        return true;
    }
    return false;
}

const Base::Placement Robot6AxisObject::getJointTransformation(const int jntID) const
{
    Base::Placement result;
    auto kin_JntPose = m_kinematicModel.getJointPose(jntID);
    auto buffer = getCurrentBasePose().toMatrix() * kin_JntPose.toMatrix();
    result.fromMatrix(buffer);
    return result;
}

bool Robot6AxisObject::setupJointChain(const std::vector<Base::Placement> &poseVec)
{
    return m_kinematicModel.setupJointChain(poseVec);
}

void Robot6AxisObject::updateAxisValues()
{
    MainAxisValues.setValues(m_kinematicModel.getJointAngles());
}

const std::vector<double> Robot6AxisObject::getAxisRunningSpeeds() const
{
    auto speedLimits = AxisRatedSpeed.getValues();
    auto speedRatio = AxisSpeedRatio.getValue();
    std::vector<double> result = std::vector<double>(speedLimits.size(), 0.0);
    for(auto i = 0; i<speedLimits.size(); i++){
        result[i] = speedLimits[i] * speedRatio;
    }
    return result;
}

bool Robot6AxisObject::setRobotPose(const RobotPose &t_Pose)
{
    bool success = true;

    Base::Placement t_CartPose;
    t_CartPose.setPosition(Base::Vector3d(t_Pose.PoseData[0],
                                          t_Pose.PoseData[1],
                                          t_Pose.PoseData[2]));
    Base::Rotation t_Rot;
    t_Rot.setYawPitchRoll(t_Pose.PoseData[3],
                          t_Pose.PoseData[4],
                          t_Pose.PoseData[5]);
    t_CartPose.setRotation(t_Rot);

    switch(t_Pose.CordInfo.first){
    case CordType::ACS:
        success &= setAxisValues(t_Pose.PoseData);
        break;
    case CordType::MCS:
    {
        success &= setRobotTipPose(t_CartPose,CoordOrigin::Robot);
        std::vector<double> extAxisVals = std::vector<double>(2,0.0);
        extAxisVals[0] = t_Pose.PoseData[6];
        extAxisVals[1] = t_Pose.PoseData[7];
        ExtrAxisValues.setValues(extAxisVals);
    }
        break;
    case CordType::TCS:{
        uint toolIndex = t_Pose.CordInfo.second;
        auto toolTrans = getToolFrameTrans(toolIndex);
        Base::Placement t_FlanPose = t_CartPose * toolTrans.inverse();
        success &= setRobotTipPose(t_FlanPose,CoordOrigin::Flan);
        std::vector<double> extAxisVals = std::vector<double>(2,0.0);
        extAxisVals[0] = t_Pose.PoseData[6];
        extAxisVals[1] = t_Pose.PoseData[7];
        ExtrAxisValues.setValues(extAxisVals);
    }
        break;
    case CordType::WCS:{
        success &= setRobotTipPose(t_CartPose,CoordOrigin::World);
        std::vector<double> extAxisVals = std::vector<double>(2,0.0);
        extAxisVals[0] = t_Pose.PoseData[6];
        extAxisVals[1] = t_Pose.PoseData[7];
        ExtrAxisValues.setValues(extAxisVals);
    }
        break;

    case CordType::PCS:
        break;
    }

    return success;
}

const RobotPose Robot6AxisObject::getRobotPose(const CordType &t_Type) const
{
    RobotPose c_Pose;
    CompPose t_CompPose;
    switch(t_Type){
    case CordType::ACS:
        c_Pose.PoseData = getJointAngles();
        c_Pose.CordInfo.first = CordType::ACS;
        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
        break;
    case CordType::MCS:
        t_CompPose.first = getCurrentTipPose(CoordOrigin::Robot);
        t_CompPose.second.first = ExtrAxisValues.getValues()[0];
        t_CompPose.second.second = ExtrAxisValues.getValues()[1];
        c_Pose.setPoseData(t_CompPose);
        c_Pose.CordInfo.first = CordType::MCS;
        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
        break;
    case CordType::TCS:
        t_CompPose.first = getCurrentTipPose();
        t_CompPose.second.first = ExtrAxisValues.getValues()[0];
        t_CompPose.second.second = ExtrAxisValues.getValues()[1];
        c_Pose.setPoseData(t_CompPose);
        c_Pose.CordInfo.first = CordType::TCS;
        c_Pose.CordInfo.second = CurrentToolIndex.getValue();
        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
        break;
    case CordType::WCS:
        t_CompPose.first = getCurrentTipPose();
        t_CompPose.second.first = ExtrAxisValues.getValues()[0];
        t_CompPose.second.second = ExtrAxisValues.getValues()[1];
        c_Pose.setPoseData(t_CompPose);
        c_Pose.CordInfo.first = CordType::WCS;
        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::World);
        break;
    }

    return c_Pose;
}

 uint Robot6AxisObject::getJointNumbers() const
{
    return m_kinematicModel.getJointNumbers() + ExtrAxisValues.getValues().size();
}

const std::vector<double> Robot6AxisObject::getJointAngles() const
{
    auto result = m_kinematicModel.getJointAngles();
    for(auto val : ExtrAxisValues.getValues())
        result.push_back(val);
    return result;
}

void Robot6AxisObject::setAttachable(bool attachable)
{
    MoveWithAttachedBody.setValue(attachable);
}

void Robot6AxisObject::setBaseToSelectedFaceCenter()
{
//    auto targetEntity = dynamic_cast<Part::Feature *>(LinkedFaceFeature.getValue());
//    if (targetEntity != nullptr)
//        targetEntity->setAttachedObj(this);

    auto newCenter = CAD_Utility::calculateLinkedFaceCenter(LinkedFaceFeature);
    Pose_Reference.setValue(Base::Placement(newCenter.toMatrix()));
}



