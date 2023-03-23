// Created By Yixiao 2022-04-16

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "MechanicRobot.h"
#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicRobot, Robot::MechanicBase)


MechanicRobot::MechanicRobot()
{
    ADD_PROPERTY_TYPE(EnableArmConfiguration ,(false), "Kinematics",Prop_None,"Identify is using configuration constraint while calc IK results");
    ADD_PROPERTY_TYPE(ConnectedExtAxis,(""),"Property",Prop_None,"The Name of Attached External Axis");
    ADD_PROPERTY_TYPE(TeachCoordIndex,(0),"Interactive",Prop_None, "Current Teach Coord Index");
    ADD_PROPERTY_TYPE(InteractiveDraggerOn,(false),"Interactive",Prop_None, "Trigger On Interactive Dragger");
    ADD_PROPERTY_TYPE(CurrentToolIndex,(0),"Tool Setup", Prop_None,"Current ID of Selected Tool");
    m_kinematicModel.setMechanicType(MechanicType::M_Robot);
    m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
}

MechanicRobot::~MechanicRobot()
{
}


void MechanicRobot::onChanged(const Property* prop)
{
   MechanicBase::onChanged(prop);
   if(prop == &AxisValues){
        // Change RobotState By set Joint Values
        if(!flag_updateTcpPose){
            flag_updateJntVals = true;
            // Update RobotModel
            m_kinematicModel.setJointAngles(AxisValues.getValues());
            Pose_Tip.setValue(getCurrentTipPose(CoordOrigin::World));
            Pose_Flan.setValue(getCurrentFlanPose(CoordOrigin::World));
            updateAssembledToolPose();
            flag_updateJntVals = false;
        }
        // Change RobotState By set TCP pose
        else{
            m_kinematicModel.updateJointFrames();
        }
   }
   else if(prop == &Pose_Tip && !flag_updateJntVals){
        // Update KinematicModel JntVals
       auto tipToolTrans_inv = getToolTipTranslation().toMatrix();
       tipToolTrans_inv.inverse();
       Pose_Flan.setValue(Pose_Tip.getValue().toMatrix() * tipToolTrans_inv);
       if(m_kinematicModel.setTo(Pose_Flan.getValue())){
            // Update JntVals to AxisValues
            flag_updateTcpPose = true;
            updateAxisValues();
            updateAssembledToolPose();
            flag_updateTcpPose = false;
        }
   }
   else if(prop == &UseTracIK){
        if(UseTracIK.getValue())
            m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
        else
            m_kinematicModel.setIKsolverType(IK_SolverType::KDL_IK);
   }
}

void MechanicRobot::onDocumentRestored()
{
    MechanicBase::onDocumentRestored();
}

void MechanicRobot::updateRobotConfiguration()
{
//    ConfigType t_Type;
//    if(Wrist_NonFlip.getValue()){
//        if(ForeArm_onRight.getValue()){
//            if(Elbow_Upward.getValue())
//                t_Type = ConfigType::NRU;
//            else
//                t_Type = ConfigType::NRD;
//        }
//        else{
//            if(Elbow_Upward.getValue())
//                t_Type = ConfigType::NLU;
//            else
//                t_Type = ConfigType::NLD;
//        }
//    }else{
//        if(ForeArm_onRight.getValue()){
//            if(Elbow_Upward.getValue())
//                t_Type = ConfigType::FRU;
//            else
//                t_Type = ConfigType::FRD;
//        }
//        else{
//            if(Elbow_Upward.getValue())
//                t_Type = ConfigType::FLU;
//            else
//                t_Type = ConfigType::FLD;
//        }
//    }
//    if(EnableArmConfiguration.getValue())
//       m_kinematicModel.setKinematicModelConfig(t_Type);
//    else
//       m_kinematicModel.setKinematicModelConfig(ConfigType::NON);
}


bool MechanicRobot::setRobotTipPose(const Base::Placement &n_TipPose, CoordOrigin pose_Coord, Base::Placement origin_Pose)
{
    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getOriginPose().inverse();
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
                     origin_Pose.toMatrix() *
                     n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
        }
            break;
    }

    bool success = false;
    if(m_kinematicModel.setTo(targetPose)){
        success = true;
        Pose_Tip.setValue(targetPose);
    }

    return success;
}

const Base::Placement MechanicRobot::getTeachDraggerPose() const
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

const Base::Placement MechanicRobot::getToolTipTranslation() const
{
    return getToolFrameTrans(CurrentToolIndex.getValue());
}

const Base::Placement MechanicRobot::getToolFrameTrans(const uint ToolID) const
{
    Base::Placement toolTrans;
    auto c_ToolIter = m_AssembledTools.find(ToolID);
     if(c_ToolIter==m_AssembledTools.end()||c_ToolIter->second == nullptr){
         return toolTrans;
     }
     return c_ToolIter->second->getTransform_Mount2Front();
}

const Base::Placement MechanicRobot::getCurrentTipPose(const CoordOrigin &ref_Origin) const
{
    Base::Placement result;
    result = getCurrentFlanPose(ref_Origin).toMatrix() * getToolTipTranslation().toMatrix();
    return result;
}

const Base::Placement MechanicRobot::getCurrentFlanPose(const CoordOrigin &ref_Origin) const
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
    default:
        break;
    }
    return c_flanPose;
}

bool MechanicRobot::TorchAssembled()
{
    for(const auto& iter : m_AssembledTools){
        if(iter.second->getToolType() == ToolType::WeldTorch)
            return true;
    }
    return false;
}


bool MechanicRobot::installTool(const char *tool_Name)
{
    if(tool_Name == nullptr)
        return false;
    auto t_ToolPtr = static_cast<Robot::ToolObject*>(getDocument()->getObject(tool_Name));
    if(t_ToolPtr == nullptr)
        return false;
    for(const auto& t_ToolInfo : m_AssembledTools){
        if(t_ToolInfo.second->_ToolType.getValue() == t_ToolPtr->_ToolType.getValue())
            return false;
    }

//    auto dynProp = t_ToolPtr->addDynamicProperty("App::PropertyLinkSubList", "Link");
//    App::PropertyLinkSubList* t_LinkPtr = static_cast<App::PropertyLinkSubList*>(dynProp);
//    t_LinkPtr->setValues(std::vector<App::DocumentObject*>({this, t_ToolPtr}),
//                         std::vector<std::string>({"Pose_Flan","Pose_Mount"}));

    m_AssembledTools.insert(std::make_pair(m_AssembledTools.size(),t_ToolPtr));
    updateAssembledToolPose();
    return true;
}

void MechanicRobot::uninstallTool(const char *tool_Name)
{
    if(tool_Name == nullptr)
        return;
    for(auto t_Iter = m_AssembledTools.begin(); t_Iter!=m_AssembledTools.end(); t_Iter++){
        if(std::strcmp(t_Iter->second->getNameInDocument(), tool_Name) == 0)
            m_AssembledTools.erase(t_Iter);
    }
}

void MechanicRobot::updateAssembledToolPose()
{
    for(auto t_ToolIter : m_AssembledTools){
        t_ToolIter.second->Pose_Mount.setValue(Pose_Flan.getValue());
    }
}

void MechanicRobot::setCurrentToolType(const ToolType &t_Type)
{

}

void MechanicRobot::setCurrentToolActive(bool activated)
{
}

ToolType MechanicRobot::getCurrentTool() const
{
    ToolType c_Type = ToolType::Undefined;
    return c_Type;
}

std::vector<DocumentObject *> MechanicRobot::getChildrenList() const
{
    std::vector<DocumentObject *> children;
    for(const auto& t_ToolPtr : m_AssembledTools){
        children.push_back(t_ToolPtr.second);
    }
    return children;
}


const Base::Placement MechanicRobot::getCurrentBasePose() const
{
    return Base::Placement(Pose_Reference.getValue() * Trans_Ref2Base.getValue());
}

const Base::Placement MechanicRobot::getSelectedFeatureCenter() const
{
    // TODO: Consider Edge Center as well
    return CAD_Utility::calculateLinkedFaceCenter(LinkedFaceFeature);
}

void MechanicRobot::setTeachCoordType(const TeachCoord &t_coord)
{
    m_TeachCoord = t_coord;
    TeachCoordIndex.setValue(t_coord);
}

void MechanicRobot::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    m_kinematicModel.Save(writer);
}

void MechanicRobot::Restore(Base::XMLReader &reader)
{
    flag_updateJntVals = true;
    App::GeoFeature::Restore(reader);
    m_kinematicModel.Restore(reader);
    flag_updateJntVals = false;
}


bool MechanicRobot::setRobotPose(const RobotPose &t_Pose)
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
        success &= setJointAngles(t_Pose.PoseData);
        break;
    case CordType::MCS:
    {
        success &= setRobotTipPose(t_CartPose,CoordOrigin::Robot);
    }
        break;
    case CordType::TCS:{
        uint toolIndex = t_Pose.CordInfo.second;
        auto toolTrans = getToolFrameTrans(toolIndex);
        Base::Placement t_FlanPose = t_CartPose * toolTrans.inverse();
        success &= setRobotTipPose(t_FlanPose,CoordOrigin::Flan);
    }
        break;
    case CordType::WCS:{
        success &= setRobotTipPose(t_CartPose,CoordOrigin::World);
    }
        break;

    case CordType::PCS:
        break;
    }

    return success;
}

const RobotPose MechanicRobot::getRobotPose(const CordType &t_Type) const
{
    RobotPose c_Pose;
    CompPose t_CompPose;
//    switch(t_Type){
//    case CordType::ACS:
//        c_Pose.PoseData = getJointAngles();
//        c_Pose.CordInfo.first = CordType::ACS;
//        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
//        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
//        break;
//    case CordType::MCS:
//        t_CompPose.first = getCurrentTipPose(CoordOrigin::Robot);
//        t_CompPose.second.first = 0;
//        t_CompPose.second.second = 0;
//        c_Pose.setPoseData(t_CompPose);
//        c_Pose.CordInfo.first = CordType::MCS;
//        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
//        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
//        break;
//    case CordType::TCS:
//        t_CompPose.first = getCurrentTipPose();
//        t_CompPose.second.first = 0;
//        t_CompPose.second.second = 0;
//        c_Pose.setPoseData(t_CompPose);
//        c_Pose.CordInfo.first = CordType::TCS;
//        c_Pose.CordInfo.second = CurrentToolIndex.getValue();
//        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
//        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::Robot);
//        break;
//    case CordType::WCS:
//        t_CompPose.first = getCurrentTipPose();
//        t_CompPose.second.first = 0;
//        t_CompPose.second.second = 0;
//        c_Pose.setPoseData(t_CompPose);
//        c_Pose.CordInfo.first = CordType::WCS;
//        c_Pose.ConfigID = (uint)m_kinematicModel.getConfigType();
//        c_Pose.FlanPose = getCurrentFlanPose(CoordOrigin::World);
//        break;
//    }

    return c_Pose;
}

void MechanicRobot::setTipPoseByDraggerPose(const Base::Placement &n_DraggerPose)
{
    Base::Placement n_TipPose = n_DraggerPose;

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

    auto base_inv = getOriginPose().inverse();
    auto tipPose_RbtFrame = base_inv * n_TipPose;
//    Base::Placement toolTrans = getToolTipTranslation();
//    Pose_Tip.setValue(Base::Placement(tipPose_RbtFrame.toMatrix()*
//                                      toolTrans.inverse().toMatrix()));
    Pose_Tip.setValue(tipPose_RbtFrame);
}

void MechanicRobot::setTipPoseByDiff(const Base::Placement &movement)
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
        Pose_Tip.setValue(Base::Placement(getCurrentBasePose().inverse().toMatrix() *
                                            trans_stp.toMatrix()*
                                            trans_org.toMatrix()*
                                            rotat_stp.toMatrix()*
                                            rotat_org.toMatrix()/**
                                            toolTrans.inverse().toMatrix()*/));
    }
        break;
    case TeachCoord::RobotBase:{
        c_TipPose = getCurrentTipPose(CoordOrigin::Robot);
        Base::Placement trans_org = Base::Placement(c_TipPose.getPosition(), Base::Rotation());
        Base::Placement rotat_org = Base::Placement(Base::Vector3d(), c_TipPose.getRotation());
        Pose_Tip.setValue(Base::Placement(trans_stp.toMatrix()*
                                           trans_org.toMatrix()*
                                           rotat_stp.toMatrix()*
                                           rotat_org.toMatrix()/**
                                           toolTrans.inverse().toMatrix()*/));
    }
        break;
    case TeachCoord::Tip:
        c_TipPose = getCurrentTipPose(CoordOrigin::World);
        Base::Placement trans_org = Base::Placement(c_TipPose.getPosition(), Base::Rotation());
        Base::Placement rotat_org = Base::Placement(Base::Vector3d(), c_TipPose.getRotation());
        Pose_Tip.setValue(Base::Placement(getCurrentBasePose().inverse().toMatrix() *
                                            trans_org.toMatrix()*
                                            rotat_org.toMatrix()*
                                            trans_stp.toMatrix()*
                                            rotat_stp.toMatrix()/**
                                            toolTrans.inverse().toMatrix()*/));
        break;
    }
}

void MechanicRobot::moveToSelectedFaceCenter()
{
    auto newCenter = CAD_Utility::calculateLinkedFaceCenter(LinkedFaceFeature);
    Pose_Reference.setValue(Base::Placement(newCenter.toMatrix()));
}



