// Created By Yixiao 2022-08-12

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>
#include "MechanicGroup.h"

#define RbtAxisNum 8
#define ExtAxisNum 8

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicGroup, App::GeoFeature)


MechanicGroup::MechanicGroup()
:flag_updateJntVals(false)
{
    ADD_PROPERTY_TYPE(Activated,(false),"Property",Prop_None,"Is robot activated");
    ADD_PROPERTY_TYPE(Visiable,(true),"Property",Prop_None,"Is Robot Visiable");
    ADD_PROPERTY_TYPE(ActiveRobotIndex,(0), "Property", Prop_None, "Active Robot Index");

    ADD_PROPERTY_TYPE(File_URDF,(0),"Files",Prop_None,"Included file with robot definition about robot model");
    ADD_PROPERTY_TYPE(File_Mesh,(0),"Files",Prop_None,"Included file with mesh path definition of the robot links");

    ADD_PROPERTY_TYPE(Pose_GroupActiveFlan,(Base::Placement()),"Kinematics",Prop_None,"Tcp Pose in World Coord");
    ADD_PROPERTY_TYPE(GroupHomePose ,(0.0), "Kinematics",Prop_None,"Axis position for home");

    ADD_PROPERTY_TYPE(JoinMovement, (false), "Kinematics", Prop_None, "Kinematic Movement");
    ADD_PROPERTY_TYPE(AxisValues, (0.0), "Kinematics", Prop_None, "Axis angle of all Axis");
    AxisValues.setSize(24);

    ADD_PROPERTY_TYPE(TeachCoordIndex,(0),"",Prop_None, "Current Teach Coord Index");

    ADD_PROPERTY(LinkedRobotName_1,(""));
    ADD_PROPERTY(LinkedRobotName_2,(""));
    ADD_PROPERTY(LinkedPoserNames,(""));
    ADD_PROPERTY(ActiveToolIndex,(0));
    ActiveToolIndex.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(InteractiveTeach,(false));
    InteractiveTeach.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(NetworkConnected, (false));

    m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
//    m_kinematicModel.setMechType(MechanicType::M_Group);
}

MechanicGroup::~MechanicGroup()
{
}

DocumentObjectExecReturn *MechanicGroup::execute()
{
    return StdReturn;
}

DocumentObjectExecReturn *MechanicGroup::recompute()
{
    m_kinematicModel.updateJointFrames();
    return this->execute();
}

short MechanicGroup::mustExecute(void) const
{
    return 0;
}

PyObject *MechanicGroup::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void MechanicGroup::onChanged(const Property* prop)
{
    if(prop == &LinkedRobotName_1){
        auto t_RobotPtr = static_cast<Robot6AxisObject*>(getDocument()->getObject(LinkedRobotName_1.getValue()));
        if(t_RobotPtr!=nullptr){
            m_LinkedRobot_1 = t_RobotPtr;
        }
        else{
            m_LinkedRobot_1 = nullptr;
        }
   }
   else if(prop == &LinkedRobotName_2){
        auto t_RobotPtr = static_cast<Robot6AxisObject*>(getDocument()->getObject(LinkedRobotName_2.getValue()));
        if(t_RobotPtr!=nullptr){
            m_LinkedRobot_2 = t_RobotPtr;
        }
        else{
            m_LinkedRobot_2 = nullptr;
        }
   }
    else if(prop == &ActiveRobotIndex){
        auto t_Index = ActiveRobotIndex.getValue();
        if(t_Index == 1 && m_LinkedRobot_1!=nullptr){
            m_ActiveRobotPtr = m_LinkedRobot_1;
        }
        if(t_Index == 2 && m_LinkedRobot_2!=nullptr){
            m_ActiveRobotPtr = m_LinkedRobot_2;
        }

        if(m_ActiveRobotPtr != nullptr){
//            Activated.setValue(false);
//            Activated.setValue(true);
        }

    }
   else if(prop == &LinkedPoserNames){
        updateLinkedPositionerList(LinkedPoserNames.getValues());
   }
   else if(prop == &AxisValues){
       if(JoinMovement.getValue()){
           // Change RobotState By set Joint Values
           if(!flag_updateTcpPose){
               flag_updateJntVals = true;
               m_kinematicModel.setJointAngles(AxisValues.getValues());
               Pose_GroupActiveFlan.setValue(getGroupFlanPose(CoordOrigin::World));
               flag_updateJntVals = false;
           }
           // Change RobotState By set TCP pose
           else{
               m_kinematicModel.updateJointFrames();
           }
       }
       else{
           updateRobotAxisValues();
           updateExternalAxisValue();
       }
    }

    else if(prop == &Pose_GroupActiveFlan && !flag_updateJntVals){
       if(JoinMovement.getValue()){
           if(m_kinematicModel.setTo(Base::Placement(Pose_GroupActiveFlan.getValue()))){
               flag_updateTcpPose = true;
               updateAxisValues();
               flag_updateTcpPose = false;
           }
       }
       else{
           if(m_ActiveRobotPtr)
               m_ActiveRobotPtr->Pose_Flan.setValue(Pose_GroupActiveFlan.getValue());
           updateAxisValues();
       }
    }

    App::GeoFeature::onChanged(prop);
}

void MechanicGroup::onDocumentRestored()
{
    App::GeoFeature::onDocumentRestored();
    flag_restoringObject = false;
    if(m_LinkedRobot_1!=nullptr)
        m_ActiveRobotPtr = m_LinkedRobot_1;
    if(m_LinkedRobot_2!=nullptr)
        m_ActiveRobotPtr = m_LinkedRobot_2;
}

void MechanicGroup::updateAxisProperties()
{
}

bool MechanicGroup::setTipPose(const Base::Placement &n_TipPose,
                               CoordOrigin pose_Coord,
                               Base::Placement coordOrigin_Pose)
{
    if(JoinMovement.getValue()){
        Base::Placement toolTrans = getToolTipTranslation();
        auto base_inv = getActiveRobotBase().inverse();
        Base::Placement t_FlanPose;
        switch(pose_Coord){
            case CoordOrigin::World:
            t_FlanPose = base_inv.toMatrix() *
                         n_TipPose.toMatrix() *
                         toolTrans.inverse().toMatrix();
                break;
            case CoordOrigin::Robot:
            t_FlanPose = n_TipPose.toMatrix() *
                         toolTrans.inverse().toMatrix();
                break;

            case CoordOrigin::Flan:
            t_FlanPose = n_TipPose;
                break;

            case CoordOrigin::Object:{
            t_FlanPose = base_inv.toMatrix() *
                         coordOrigin_Pose.toMatrix() *
                         n_TipPose.toMatrix() *
                         toolTrans.inverse().toMatrix();
            }
                break;
        }

        if(m_kinematicModel.setTo(t_FlanPose)){
            Pose_GroupActiveFlan.setValue(t_FlanPose);
            return true;
        }
    }
    else{
        if(m_ActiveRobotPtr)
            return m_ActiveRobotPtr->setRobotTipPose(n_TipPose, pose_Coord,coordOrigin_Pose);
    }

    return false;
}

const Base::Placement MechanicGroup::getGroupTipPose(CoordOrigin ref_Origin,
                                                     Base::Placement origin_Pose) const
{
    Base::Placement result, transT;
    if(JoinMovement.getValue()){
        transT = getToolTipTranslation();
        result = Base::Placement(getGroupFlanPose(ref_Origin,origin_Pose).toMatrix() *
                                 transT.toMatrix());
        return result;
    }
    else{
        if(m_ActiveRobotPtr)
            return m_ActiveRobotPtr->getCurrentTipPose(ref_Origin,origin_Pose);
    }


}

const Base::Placement MechanicGroup::getGroupFlanPose(const CoordOrigin &ref_Origin,
                                                        Base::Placement origin_Pose) const
{
    if(JoinMovement.getValue()){
        Base::Placement c_flanPose;
        if(!m_kinematicModel.initialized())
            return c_flanPose;

        Base::Placement abs_flanPose = m_kinematicModel.getTcp();

        switch(ref_Origin){
        case CoordOrigin::World:
            c_flanPose= Base::Placement(getActiveRobotBase().toMatrix() *
                                        abs_flanPose.toMatrix());
            break;
        case CoordOrigin::Robot:
            c_flanPose = abs_flanPose;
            break;
        case CoordOrigin::Flan:
            c_flanPose = Base::Placement();
            break;
        case CoordOrigin::Object:{
//            auto originPose_inv = origin_Pose;
//            originPose_inv.inverse();
//            c_flanPose= Base::Placement(originPose_inv.toMatrix() * getCurrentFlanPose(CoordOrigin::World).toMatrix() );
            break;
        }
        default:
            break;
        }
        return c_flanPose;
    }else{
        if(m_ActiveRobotPtr)
            return m_ActiveRobotPtr->getCurrentFlanPose(ref_Origin, origin_Pose);
    }

}


void MechanicGroup::setTipPoseByDraggerPose(const Base::Placement &n_DraggerPose)
{
    Base::Placement n_TipPose = n_DraggerPose;

    switch(m_TeachCoord){
    case TeachCoord::World:{
        n_TipPose.setRotation(getGroupTipPose(CoordOrigin::World).getRotation());
    }
        break;
    case TeachCoord::RobotBase:{
        n_TipPose.setRotation(getGroupTipPose(CoordOrigin::Robot).getRotation());
    }
        break;
    case TeachCoord::Tip:
        break;
    }

    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getActiveRobotBase().inverse();

    auto tipPose_RbtFrame = base_inv * n_TipPose;
    Pose_GroupActiveFlan.setValue(Base::Placement(tipPose_RbtFrame.toMatrix()*
                                           toolTrans.inverse().toMatrix()));
}

void MechanicGroup::setTipPoseByDiff(const Base::Placement &difference)
{
//    c_TipPose;
    Base::Placement c_TipPoseW = getGroupTipPose(CoordOrigin::World);
    Base::Placement c_TipPoseR = getGroupTipPose(CoordOrigin::Robot);
    Base::Placement toolTrans = getToolTipTranslation();
    Base::Placement trans_diff = Base::Placement(difference.getPosition(), Base::Rotation());
    Base::Placement rotat_diff = Base::Placement(Base::Vector3d(), difference.getRotation());

    Base::Placement trans_orig = Base::Placement(c_TipPoseW.getPosition(), Base::Rotation());
    Base::Placement rotat_orig = Base::Placement(Base::Vector3d(), c_TipPoseW.getRotation());

    Base::Placement t_FlanPose;

    switch(m_TeachCoord){
    case TeachCoord::World:{
        t_FlanPose = Base::Placement(getActiveRobotBase().inverse().toMatrix() *
                                     trans_orig.toMatrix()*
                                     trans_diff.toMatrix()*
                                     rotat_orig.toMatrix()*
                                     rotat_diff.toMatrix()*
                                     toolTrans.inverse().toMatrix());

    }
        break;
    case TeachCoord::RobotBase:{
        Base::Placement trans_orig = Base::Placement(c_TipPoseR.getPosition(), Base::Rotation());
        Base::Placement rotat_orig = Base::Placement(Base::Vector3d(), c_TipPoseR.getRotation());
        t_FlanPose = Base::Placement(trans_diff.toMatrix()*
                                     trans_orig.toMatrix()*
                                     rotat_diff.toMatrix()*
                                     rotat_orig.toMatrix()*
                                     toolTrans.inverse().toMatrix());
    }
        break;
    case TeachCoord::Tip:
        t_FlanPose = Base::Placement(getActiveRobotBase().inverse().toMatrix() *
                                     trans_orig.toMatrix()*
                                     rotat_orig.toMatrix()*
                                     trans_diff.toMatrix()*
                                     rotat_diff.toMatrix()*
                                     toolTrans.inverse().toMatrix());
        break;
    }

    Pose_GroupActiveFlan.setValue(t_FlanPose);
}

bool MechanicGroup::hasTorch()
{
    if(m_ActiveRobotPtr)
        return m_ActiveRobotPtr->hasTorch();
    return false;
}

bool MechanicGroup::hasScanner()
{
    if(m_ActiveRobotPtr)
        return m_ActiveRobotPtr->hasScanner();
    return false;
}

const Base::Placement MechanicGroup::getToolTipTranslation() const
{
    Base::Placement toolTrans;
    if(m_ActiveRobotPtr)
        toolTrans = m_ActiveRobotPtr->getToolTipTranslation();
    return toolTrans;
}

void MechanicGroup::setCurrentToolType(const ToolType &t_Type)
{
    if(m_ActiveRobotPtr)
        m_ActiveRobotPtr->setCurrentToolType(t_Type);
    ActiveToolIndex.setValue((int)t_Type);
}

void MechanicGroup::setCurrentToolActive(bool activated)
{
    if(m_ActiveRobotPtr)
        m_ActiveRobotPtr->setCurrentToolActive(activated);
}

const ToolType MechanicGroup::getCurrentTool() const
{
    ToolType c_Type = ToolType::NoTool;
    if(m_ActiveRobotPtr)
        c_Type = m_ActiveRobotPtr->getCurrentTool();
    return c_Type;
}

std::vector<DocumentObject *> MechanicGroup::getChildrenList() const
{
    std::vector<DocumentObject *> result;
    if(m_LinkedRobot_1)
        result.push_back(m_LinkedRobot_1);
    if(m_LinkedRobot_2)
        result.push_back(m_LinkedRobot_2);
    for(auto t_Poser : m_ExternalAxisList)
        result.push_back(t_Poser);
    return result;
}

const Base::Placement MechanicGroup::getTeachDraggerPose() const
{
    auto result = getGroupTipPose(CoordOrigin::World);
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

const Base::Placement MechanicGroup::getCurrentBasePose() const
{
//    if(JoinMovement.getValue())
//        return Pose_Base.getValue();
//    else
        return getActiveRobotBase();
}

bool MechanicGroup::setCurrentPoseAsHome()
{
    if(JoinMovement.getValue()){
        auto c_jntVals = m_kinematicModel.getJointAngles();
        if(c_jntVals.size() != GroupHomePose.getValues().size())
            return false;
        GroupHomePose.setValues(c_jntVals);
        return true;
    }else{
        GroupHomePose.setValues(AxisValues.getValues());
//        if(m_LinkedRobot_1)
//            m_LinkedRobot_1->setCurrentPoseAsHome();
//        if(m_LinkedRobot_2)
//            m_LinkedRobot_2->setCurrentPoseAsHome();
//        for(auto t_Poser : m_ExternalAxisList){
//            t_Poser->setCurrentPoseAsHome();
//        }
    }
}

bool MechanicGroup::restoreHomePose()
{
    bool success = true;
    if(JoinMovement.getValue()){
        if(m_kinematicModel.setJointAngles(GroupHomePose.getValues())){
            updateAxisValues();
            return success;
        }
        return false;
    }else{
        AxisValues.setValues(GroupHomePose.getValues());
//        if(m_LinkedRobot_1)
//            success &= m_LinkedRobot_1->restoreHomePose();
//        if(m_LinkedRobot_2)
//            success &= m_LinkedRobot_2->restoreHomePose();
//        for(auto t_Poser : m_ExternalAxisList)
//            success &= t_Poser->restoreHomePose();
    }
    return success;
}

const Base::Placement MechanicGroup::getActiveRobotBase() const
{
    if(m_ActiveRobotPtr)
        return m_ActiveRobotPtr->getCurrentBasePose();
    Base::Console().Message("Warning: NO ACTIVE ROBOT assigned, return origin Point as GroupBase!\n");
    return Base::Placement();
}

const float MechanicGroup::getJointMaxAngle(const uint t_jntID) const
{
    if(JoinMovement.getValue())
        return m_kinematicModel.getMaxAngle(t_jntID);
    else{
        if(0<=t_jntID && t_jntID<8){
            return m_LinkedRobot_1!=nullptr?m_LinkedRobot_1->getJointMaxAngle(t_jntID):0;
        }
        else if(8<= t_jntID && t_jntID<16){
            return m_LinkedRobot_2!=nullptr?m_LinkedRobot_2->getJointMaxAngle(t_jntID-8):0;
        }
        else{
            auto t_ID = t_jntID - 16;
            for(auto t_Poser : m_ExternalAxisList){
                if(t_ID<t_Poser->getJointNumbers()){
                    return t_Poser->getJointMaxAngle(t_ID);
                }
                else{
                    t_ID-=t_Poser->getJointNumbers();
                }
            }
        }
    }
    return -1;
}

const float MechanicGroup::getJointMinAngle(const uint t_jntID) const
{
    if(JoinMovement.getValue())
        return m_kinematicModel.getMinAngle(t_jntID);
    else{
        if(0<=t_jntID && t_jntID<8){
            return m_LinkedRobot_1!=nullptr?m_LinkedRobot_1->getJointMinAngle(t_jntID):0;
        }
        else if(8<= t_jntID && t_jntID<16){
            return m_LinkedRobot_2!=nullptr?m_LinkedRobot_2->getJointMinAngle(t_jntID-8):0;
        }
        else{
            auto t_ID = t_jntID - 16;
            for(auto t_Poser : m_ExternalAxisList){
                if(t_ID<t_Poser->getJointNumbers()){
                    return t_Poser->getJointMinAngle(t_ID);
                }
                else{
                    t_ID-=t_Poser->getJointNumbers();
                }
            }
        }
    }
    return -1;
}

const string MechanicGroup::getActiveOperatorName() const
{
    auto c_RbtID = ActiveRobotIndex.getValue();
    if(c_RbtID == 1 && m_LinkedRobot_1!=nullptr)
        return std::string(m_LinkedRobot_1->getNameInDocument());
    if(c_RbtID == 2 && m_LinkedRobot_2!=nullptr)
        return std::string(m_LinkedRobot_2->getNameInDocument());
    return std::string("No Robot");
}

void MechanicGroup::setActiveRobot(const uint t_ID)
{
    if(t_ID == 1 || t_ID == 2){
        ActiveRobotIndex.setValue(t_ID);
    }
}

void MechanicGroup::changeOperator()
{
    auto c_RbtID = ActiveRobotIndex.getValue();
    if(c_RbtID<2)
        ActiveRobotIndex.setValue(c_RbtID+1);
    else
        ActiveRobotIndex.setValue(c_RbtID-1);
}

void MechanicGroup::setStationAddress(const string t_IP, const uint t_PortNum)
{
    m_ControllerIP = t_IP;
    m_ControllerPort = t_PortNum;
}

bool MechanicGroup::stationConnected()
{
    if(m_CommManager!=nullptr)
        return m_CommManager->isRobotConnected();
    return false;
}

bool MechanicGroup::connectToRobot()
{
    if(m_CommManager == nullptr){
        m_CommManager = std::make_shared<ControllerTcpConnector>(this, nullptr);
        if(!m_CommManager->flag_initialized){
            m_CommManager = nullptr;
            return false;
        }
    }
    m_CommManager->connectToRobot(QString::fromStdString(m_ControllerIP),m_ControllerPort) == 0;

    return m_CommManager->isRobotConnected();
}

void MechanicGroup::disconnectRobot()
{
    if(m_CommManager != nullptr){
        m_CommManager->disconnectFromRobot();
    }
}

bool MechanicGroup::startUpdatePose()
{
    m_CommManager->flag_stopUpdate = false;
    m_CommManager->start();
    return true;
}

void MechanicGroup::stopUpdatePose()
{
    m_CommManager->flag_stopUpdate = true;
}

void MechanicGroup::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    if(JoinMovement.getValue())
        m_kinematicModel.Save(writer);
    else{
        if(m_LinkedRobot_1)
            m_LinkedRobot_1->Save(writer);
        if(m_LinkedRobot_2)
            m_LinkedRobot_2->Save(writer);
        for(auto t_PoserPtr : m_ExternalAxisList){
            if(t_PoserPtr!=nullptr)
                t_PoserPtr->Save(writer);
        }
    }
}

void MechanicGroup::Restore(Base::XMLReader &reader)
{
    flag_restoringObject = true;
    App::GeoFeature::Restore(reader);
    if(JoinMovement.getValue()){
        flag_updateJntVals = true;
        m_kinematicModel.Restore(reader);
        flag_updateJntVals = false;
    }
}

const Robot6AxisObject *MechanicGroup::getLinkedRobot1Ptr() const
{
    return m_LinkedRobot_1;
}

const Robot6AxisObject *MechanicGroup::getLinkedRobot2Ptr() const
{
    return m_LinkedRobot_2;
}

const std::vector<MechanicDevice *> &MechanicGroup::getLinkedPositioner() const
{
    return m_ExternalAxisList;
}

void MechanicGroup::updateLinkedPositionerList(const std::vector<string> t_Names)
{
    m_ExternalAxisList.clear();
    uint emptyJnts = ExtAxisNum;
    for(auto t_Name : t_Names){
        auto t_PoserPtr = static_cast<Robot::MechanicDevice*>(getDocument()->getObject(t_Name.c_str()));
        if(t_PoserPtr!=nullptr){
            auto t_JntNbr = t_PoserPtr->getJointNumbers();
            if(t_JntNbr<emptyJnts){
                emptyJnts -= t_JntNbr;
                m_ExternalAxisList.push_back(t_PoserPtr);
            }
        }
    }
}

void MechanicGroup::updateRobotAxisValues()
{
    auto t_vals = AxisValues.getValues();
    if(m_LinkedRobot_1){
        std::vector<double> buffer;
        uint jntNum = m_LinkedRobot_1->getJointNumbers();
        for(int i = 0; i<jntNum ; i++){
            buffer.push_back(t_vals[i]);
        }
        m_LinkedRobot_1->setAxisValues(buffer);
    }
    if(m_LinkedRobot_2){
        std::vector<double> buffer;
        uint jntNum = m_LinkedRobot_2->getJointNumbers();
        for(int i = 8; i<8+jntNum ; i++){
            buffer.push_back(t_vals[i]);
        }
        m_LinkedRobot_2->setAxisValues(buffer);
    }
}

void MechanicGroup::updateExternalAxisValue()
{
    auto axisVals = AxisValues.getValues();
    if(axisVals.size()!=24)
        return;
    std::vector<double> t_vals(axisVals.begin()+16, axisVals.end());
    int c_id = 0;
    for(auto t_Poser : m_ExternalAxisList){
        auto jntNrb = t_Poser->getJointNumbers();
        t_Poser->setJointAngles(std::vector<double>(t_vals.begin()+c_id,
                                                    t_vals.begin()+c_id+jntNrb));
        c_id+=jntNrb;
    }
}


bool MechanicGroup::setJointAngles(const double angles[], size_t size)
{
    if(size!=getJointNumbers())
        return false;
    std::vector<double> t_angles;
    for(int i = 0; i<size; i++){
        t_angles.push_back(angles[i]);
    }
    AxisValues.setValues(t_angles);
    return true;
}

const std::vector<double> MechanicGroup::getGroupJointSpeed() const
{
    std::vector<double> jntSpeed = std::vector<double>(24,0.0);
    if(m_LinkedRobot_1){
        auto rbt1_JntSpeeds = m_LinkedRobot_1->getAxisRunningSpeeds();
        for(int i = 0; i<8; i++){
            jntSpeed[i] = rbt1_JntSpeeds[i];
        }
    }
    if(m_LinkedRobot_2){
        auto rbt2_JntSpeeds = m_LinkedRobot_2->getAxisRunningSpeeds();
        for(int i = 8; i<16; i++){
            jntSpeed[i] = rbt2_JntSpeeds[i-8];
        }
    }
    auto extAxis_JntSpeeds = getExtAxisSpeedLimits();
    for(int i = 16; i<24; i++){
        jntSpeed[i] = extAxis_JntSpeeds[i-16];
    }

    return jntSpeed;
}

const std::vector<double> MechanicGroup::getExtAxisSpeedLimits() const
{
    std::vector<double> jntSpeed = std::vector<double>(8,0.0);
    int c_id = 0;
    for(auto poser : m_ExternalAxisList){
        auto t_speeds = poser->getAxisRuningSpeed();
        for(int i = c_id; i<c_id+t_speeds.size(); i++){
            jntSpeed[i] = t_speeds[i-c_id];
        }
        c_id+=t_speeds.size();
    }
    return jntSpeed;
}

bool MechanicGroup::setJointAngle(int t_jntID, float jntAngle)
{
    if(t_jntID > AxisValues.getValues().size())
        return false;
    bool success = true;
    if(JoinMovement.getValue()){
        success &= m_kinematicModel.setJointAngle(t_jntID, jntAngle);
        updateAxisValues();
    }
    else{
        AxisValues.set1Value(t_jntID,jntAngle);
        AxisValues.touch();
    }
    return success;
}

const float MechanicGroup::getJointAngle(const int t_jntID) const
{
    if(JoinMovement.getValue())
       return m_kinematicModel.getJointAngle(t_jntID);
    else{
       // TODO: Need to Reconsider about jointID distribution
       return AxisValues.getValues()[t_jntID];
    }
}

const std::vector<double> MechanicGroup::getExtAxisVals() const
{
    std::vector<double> extAxis = std::vector<double>(ExtAxisNum,0.0);
    int t_ID = 0;
    for(auto t_PoserPtr : m_ExternalAxisList){
        auto t_PoserVals = t_PoserPtr->getJointAngles();
        for(int i = t_ID; i< t_ID+t_PoserVals.size() && i<ExtAxisNum; i++ )
            extAxis[i] = t_PoserVals[i-t_ID];
        t_ID+=t_PoserVals.size();
    }
    return extAxis;
}

const Base::Placement MechanicGroup::getJointTransformation(const int jntID) const
{
    Base::Placement result;
    auto kin_JntPose = m_kinematicModel.getJointPose(jntID);
    auto buffer = getActiveRobotBase().toMatrix() * kin_JntPose.toMatrix();
    result.fromMatrix(buffer);
    return result;
}

bool MechanicGroup::setupJointChain(const std::vector<Base::Placement> &poseVec)
{
    return m_kinematicModel.setupJointChain(poseVec);
}

void MechanicGroup::updateAxisValues()
{
    AxisValues.setValues(getJointAngles());
}

bool MechanicGroup::setGroupPose(const GroupPose &t_GroupPose)
{
    if(!t_GroupPose.isValid())
        return false;
    bool success = true;
    if(m_LinkedRobot_1){
        success &= m_LinkedRobot_1->setRobotPose(t_GroupPose.Pose_Rbt1);
    }
    if(m_LinkedRobot_2){
        success &= m_LinkedRobot_2->setRobotPose(t_GroupPose.Pose_Rbt2);
    }

    uint c_id = 0;
    for(auto t_Poser : m_ExternalAxisList){
        auto jntNbr = t_Poser->getJointNumbers();
        t_Poser->setJointAngles(std::vector<double>(t_GroupPose.ExtVals.begin()+c_id,
                                                    t_GroupPose.ExtVals.begin()+c_id+jntNbr));
        c_id+=jntNbr;
    }
    updateAxisValues();
    return success;
}

const GroupPose MechanicGroup::getCurrentGroupPose(const CordType& t_Type) const
{
    GroupPose c_Pose;
    if(m_LinkedRobot_1){
        c_Pose.Pose_Rbt1 = m_LinkedRobot_1->getRobotPose(t_Type);
    }
    if(m_LinkedRobot_2){
        c_Pose.Pose_Rbt2 = m_LinkedRobot_2->getRobotPose(t_Type);
    }
    c_Pose.ExtVals = getExtAxisVals();

    return c_Pose;
}

const std::vector<double> MechanicGroup::getCurrentExtAxisValue() const
{
    std::vector<double> result = std::vector<double>(8,0.0);
    auto c_JntVal = AxisValues.getValues();
    std::copy(c_JntVal.begin()+16, c_JntVal.end(), result.begin());
    return result;
}

const uint MechanicGroup::getJointNumbers() const
{
    if(JoinMovement.getValue())
        return m_kinematicModel.getJointNumbers();
    else{
//        uint totalAxisNumber = 0;
//        if(m_LinkedRobot_1)
//            totalAxisNumber+=m_LinkedRobot_1->getJointNumbers();
//        if(m_LinkedRobot_2)
//            totalAxisNumber+=m_LinkedRobot_2->getJointNumbers();
//        for(auto t_Poser : m_PositionerList)
//            totalAxisNumber+=t_Poser->getJointNumbers();
//        return totalAxisNumber;
        return RbtAxisNum*2 + ExtAxisNum;
    }
}


const std::vector<double> MechanicGroup::getJointAngles() const
{
    if(JoinMovement.getValue())
        return m_kinematicModel.getJointAngles();
    else{
        std::vector<double> t_Buffer = std::vector<double>(24,0.0);
        // Robot-1: 0-7
        if(m_LinkedRobot_1){
            auto robot1_Angles = m_LinkedRobot_1->getJointAngles();
            for(int i = 0; i<8; i++ )
                t_Buffer[i] = robot1_Angles[i];
        }
        // Robot-2: 8-16
        if(m_LinkedRobot_2){
            auto robot2_Angles = m_LinkedRobot_2->getJointAngles();
            for(int i = 8; i<16; i++ )
                t_Buffer[i] = robot2_Angles[i-8];
        }
        // Ext Axis: 16-24
        int t_ID = 16;
        for(auto t_PoserPtr : m_ExternalAxisList){
            auto t_PoserVals = t_PoserPtr->getJointAngles();
            for(int i = t_ID; i< t_ID+t_PoserVals.size(); i++ )
                t_Buffer[i] = t_PoserVals[i-t_ID];
            t_ID+=t_PoserVals.size();
        }
        return t_Buffer;
    }
}



