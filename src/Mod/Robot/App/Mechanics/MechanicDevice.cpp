
#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "MechanicDevice.h"
#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicDevice, App::GeoFeature)


MechanicDevice::MechanicDevice()
:flag_updateJntVals(false)
{
    ADD_PROPERTY_TYPE(Activated,(false),"Property",Prop_None,"Is robot activated");
    ADD_PROPERTY_TYPE(Visiable,(true),"Property",Prop_None,"Is Robot Visiable");

    ADD_PROPERTY_TYPE(File_URDF,(0),"Files",Prop_None,"Included file with robot definition about robot model");

    ADD_PROPERTY_TYPE(OriginReference,(""),"Robot Kinematic",Prop_None,"The Name of Origin Reference");
    ADD_PROPERTY_TYPE(Pose_Ref2Base,(Base::Placement()),"Robot kinematic",Prop_None,"Base placement of the robot");
    ADD_PROPERTY_TYPE(Pose_Tip,(Base::Placement()),"Robot kinematic",Prop_None,"Tcp Pose in World Coord");
    ADD_PROPERTY_TYPE(Pose_Reference,(Base::Placement()),"Robot Kinematic",Prop_None,"The Placement of Reference");
    ADD_PROPERTY_TYPE(HomePose ,(0.0), "Robot kinematic",Prop_None,"Axis position for home");
    ADD_PROPERTY_TYPE(AxisValues, (0.0), "Robot kinematic", Prop_None, "Axis angle of the robot joints");

    ADD_PROPERTY_TYPE(UpperLimits_Real, (0.0), "Robot kinematic", Prop_None, "Real Upper Limits");
    ADD_PROPERTY_TYPE(UpperLimits_Soft, (0.0), "Robot kinematic", Prop_None, "Soft Upper Limits");
    ADD_PROPERTY_TYPE(LowerLimits_Real, (0.0), "Robot kinematic", Prop_None, "Real Lower Limits");
    ADD_PROPERTY_TYPE(LowerLimits_Soft, (0.0), "Robot kinematic", Prop_None, "Soft Lower Limits");
    ADD_PROPERTY_TYPE(AxisRatedSpeed, (0.0), "Robot kinematic", Prop_None, "Max Moving Speed of each Axis");
    ADD_PROPERTY_TYPE(AxisSpeedRatio, (1.0), "Robot kinematic", Prop_None, "Speed Limits for All Axis Angles");
    ADD_PROPERTY_TYPE(isDriven, (false),"Property", Prop_Hidden,"");
    ADD_PROPERTY_TYPE(TeachCoordIndex,(0),"Property",Prop_None, "Current Teach Coord Index");

    ADD_PROPERTY_TYPE(MountedWorkPieceName,(""),"Load",Prop_None, "The Name of Mounted Object");
    ADD_PROPERTY_TYPE(MountedRobotName,(""),"Load",Prop_None, "The Name of Mounted Robot");

    ADD_PROPERTY(Editing, (false));
    ADD_PROPERTY(Path_CalibFile, (""));
    ADD_PROPERTY(DeviceType,(1));

    Placement.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(LinkedEdgeFeature, (0));
    LinkedEdgeFeature.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(LinkedFaceFeature, (0));
    LinkedFaceFeature.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(MoveWithAttachedBody,(false));
    MoveWithAttachedBody.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(InteractiveTeach,(false));
    InteractiveTeach.setStatus(App::Property::Status::Hidden, true);

    m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
}

MechanicDevice::~MechanicDevice()
{
}

DocumentObjectExecReturn *MechanicDevice::execute()
{
    return StdReturn;
}

DocumentObjectExecReturn *MechanicDevice::recompute()
{
    m_kinematicModel.updateJointFrames();
    return this->execute();
}

short MechanicDevice::mustExecute(void) const
{
    return 0;
}

PyObject *MechanicDevice::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void MechanicDevice::onChanged(const Property* prop)
{
   if(prop == &File_URDF){
        auto result = m_kinematicModel.readURDFfiles(File_URDF.getValue());
        if(!result){
            // Export Warning
        }
        else{
            auto J_Num = m_kinematicModel.getJointNumbers();
            HomePose.setSize(J_Num);
            AxisValues.setSize(J_Num);
            UpperLimits_Real.setValues(m_kinematicModel.getJointMaxAngles());
            UpperLimits_Soft.setValues(m_kinematicModel.getJointMaxAngles());
            LowerLimits_Real.setValues(m_kinematicModel.getJointMinAngles());
            LowerLimits_Soft.setValues(m_kinematicModel.getJointMinAngles());
            AxisRatedSpeed.setValues(m_kinematicModel.getJointSpeedLimits());
            m_AxisOrigins = std::vector<Base::Placement>(J_Num, Base::Placement());
        }
    }
    else if(prop == &UpperLimits_Soft){
        m_kinematicModel.setJointMaxLimits(UpperLimits_Soft.getValues());
    }
    else if(prop == &LowerLimits_Soft){
        m_kinematicModel.setJointMinLimits(LowerLimits_Soft.getValues());
    }

    else if(prop == &AxisValues){
        // Change RobotState By set Joint Values
        if(!flag_updateTcpPose){
            flag_updateJntVals = true;
            // Update RobotModel
            m_kinematicModel.setJointAngles(AxisValues.getValues());
            Pose_Tip.setValue(getCurrentFlanPose(CoordOrigin::World));
            udpateLoadPosition();
            flag_updateJntVals = false;
        }
        // Change RobotState By set TCP pose
        else{
            m_kinematicModel.updateJointFrames();
        }
    }

    else if(prop == &DeviceType){
       m_kinematicModel.setMechanicType((MechanicType)DeviceType.getValue());
    }

    App::GeoFeature::onChanged(prop);
}

void MechanicDevice::onDocumentRestored()
{
    App::GeoFeature::onDocumentRestored();
    AxisValues.setValues(AxisValues.getValues());
    if(!MountedWorkPieceName.getStrValue().empty()){
        loadWorkPiece(MountedWorkPieceName.getValue());
    }
}

bool MechanicDevice::setTipPose(const Base::Placement &n_TipPose,
                                  CoordOrigin pose_Coord,
                                  Base::Placement coordOrigin_Pose)
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
                     coordOrigin_Pose.toMatrix() *
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

const Base::Placement MechanicDevice::getCurrentTipPose(CoordOrigin ref_Origin,
                                                          Base::Placement origin_Pose) const
{
    Base::Placement result, transT;
    transT = getToolTipTranslation();
    result = Base::Placement(getCurrentFlanPose(ref_Origin,origin_Pose).toMatrix() *
                             transT.toMatrix());

    return result;
}

const Base::Placement MechanicDevice::getCurrentFlanPose(const CoordOrigin &ref_Origin,
                                                           Base::Placement origin_Pose) const
{
    Base::Placement c_flanPose;
    if(!m_kinematicModel.initialized())
        return c_flanPose;

    Base::Placement abs_flanPose = m_kinematicModel.getTcp();

    switch(ref_Origin){
    case CoordOrigin::World:
        c_flanPose= Base::Placement(getOriginPose().toMatrix() *
                                    abs_flanPose.toMatrix());
        break;
    case CoordOrigin::Robot:
        c_flanPose = abs_flanPose;
        break;
    case CoordOrigin::Flan:
        c_flanPose = Base::Placement();
        break;
    case CoordOrigin::Object:{
        auto originPose_inv = origin_Pose;
        originPose_inv.inverse();
        c_flanPose= Base::Placement(originPose_inv.toMatrix() * getCurrentFlanPose(CoordOrigin::World).toMatrix() );
    }
    default:
        break;
    }
    return c_flanPose;
}


void MechanicDevice::setTipPoseByDraggerPose(const Base::Placement &n_DraggerPose)
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

    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getOriginPose().inverse();

    auto tipPose_RbtFrame = base_inv * n_TipPose;
    Pose_Tip.setValue(Base::Placement(tipPose_RbtFrame.toMatrix()*
                                      toolTrans.inverse().toMatrix()));
}

void MechanicDevice::loadRobot(const char *robotName)
{
    if(robotName == nullptr)
        return;
    auto t_RobotPtr = static_cast<Robot::Robot6AxisObject*>(getDocument()->getObject(robotName));
    if(t_RobotPtr == nullptr)
        return;
    m_MountedRobotPtr = t_RobotPtr;
    udpateLoadPosition();
    MountedRobotName.setValue(std::string(t_RobotPtr->getNameInDocument()));
}

void MechanicDevice::unloadRobot(const char *robotName)
{
    m_MountedRobotPtr->Pose_Reference.setValue(Base::Placement());
    m_MountedRobotPtr = nullptr;
    MountedRobotName.setValue("");
}


void MechanicDevice::loadWorkPiece(const char *obejctName)
{
    if(obejctName == nullptr)
        return;
    auto t_ObjectPtr = static_cast<Robot::PlanningObject*>(getDocument()->getObject(obejctName));
    if(t_ObjectPtr == nullptr)
        return;
    m_MountedObject = t_ObjectPtr;
    udpateLoadPosition();
    MountedWorkPieceName.setValue(std::string(t_ObjectPtr->getNameInDocument()));
}

void MechanicDevice::unloadWorkPiece()
{
    m_MountedObject->Pose_Mount.setValue(Base::Placement());
    m_MountedObject = nullptr;
    MountedWorkPieceName.setValue("");
}

void MechanicDevice::udpateLoadPosition()
{
    if(m_MountedObject)
        m_MountedObject->Pose_Mount.setValue(getCurrentTipPose());
    if(m_MountedRobotPtr)
        m_MountedRobotPtr->Pose_Reference.setValue(getCurrentTipPose());
}

std::vector<DocumentObject *> MechanicDevice::getChildrenList() const
{
    std::vector<DocumentObject *> result;
    if(m_MountedObject)
        result.push_back(m_MountedObject);
    return result;
}

const Base::Placement MechanicDevice::getToolTipTranslation() const
{
    return Base::Placement();
}

const Base::Placement MechanicDevice::getTeachDraggerPose() const
{
//    auto result = getOriginPose() * m_kinematicModel.getJointPose(0);
    auto result = getCurrentTipPose(CoordOrigin::World);
//    Base::Rotation t_Rot;
//    switch (m_TeachCoord) {
//    case TeachCoord::World:
//        t_Rot = Base::Rotation();
//        break;
//    case  TeachCoord::RobotBase:
//        t_Rot = getOriginPose().getRotation();
//        break;
//    case  TeachCoord::Tip:
//        t_Rot = result.getRotation();
//    default:
//        break;
//    }
//    result.setRotation(t_Rot);
    return result;
}

bool MechanicDevice::setCurrentPoseAsHome()
{
    auto c_jntVals = m_kinematicModel.getJointAngles();
    if(c_jntVals.size() != HomePose.getValues().size())
        return false;
    HomePose.setValues(c_jntVals);
    return true;
}

bool MechanicDevice::restoreHomePose()
{
    if(m_kinematicModel.setJointAngles(HomePose.getValues())){
        updateAxisValues();
        return true;
    }
    return false;
}

const float MechanicDevice::getJointMaxAngle(const uint JointID) const
{
    return m_kinematicModel.getMaxAngle(JointID);
}

const float MechanicDevice::getJointMinAngle(const uint JointID) const
{
    return m_kinematicModel.getMinAngle(JointID);
}

const std::vector<double> MechanicDevice::getJointMaxAngles() const
{
    return m_kinematicModel.getJointMaxAngles();
}

const std::vector<double> MechanicDevice::getJointMinAngles() const
{
    return m_kinematicModel.getJointMinAngles();
}


void MechanicDevice::restoreJointLimits()
{
    m_kinematicModel.setJointMaxLimits(UpperLimits_Real.getValues());
    m_kinematicModel.setJointMinLimits(LowerLimits_Real.getValues());
}

void MechanicDevice::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    m_kinematicModel.Save(writer);
}

void MechanicDevice::Restore(Base::XMLReader &reader)
{
    App::GeoFeature::Restore(reader);
    flag_updateJntVals = true;
    m_kinematicModel.Restore(reader);
    flag_updateJntVals = false;
}

const std::vector<string> MechanicDevice::getAxisNames() const
{
    auto b_Name = std::string(getNameInDocument());
    auto names = m_kinematicModel.getJointNames();
    std::vector<string> result;
    for(auto j_Name : names){
        result.push_back(b_Name+std::string("_")+j_Name);
    }
    return result;
}

KinematicModel &MechanicDevice::getKinematicModelRef()
{
    return m_kinematicModel;
}

void MechanicDevice::setAxisOriginPose(const uint axisID, const Base::Placement &t_Pose)
{
//    if(axisID>3)
//        return;
//    Base::Placement new_Origin = t_Pose;
//    switch(axisID){
//    case 1:
//        Pose_Ref2Base.setValue(new_Origin);
//        break;
//    case 2:
//        if(t_Pose != Base::Placement()){
//            auto newTrans = Pose_Ref2Base.getValue().getPosition();
//            auto newRotat = Pose_Ref2Base.getValue().getRotation();
//            newTrans.y = t_Pose.getPosition().y;
//            new_Origin = Base::Placement(newTrans, newRotat);
//        }
//        break;
//    case 3:
//        break;
//    default:
//        break;
//    }
//    Pose_Ref2Base.setValue(new_Origin);
//    m_AxisOrigins[axisID-1] = new_Origin;
}

const Base::Placement MechanicDevice::getAxisOriginPose(const uint t_AxisID)
{
    Base::Placement t_Result;
    switch(t_AxisID){
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    }
    return t_Result;
}

const Base::Placement MechanicDevice::getOriginPose() const
{
    return Pose_Reference.getValue()*Pose_Ref2Base.getValue();
}

bool MechanicDevice::flipAxisDirection(uint jntID, bool invert)
{
    m_kinematicModel.flipJointDir(jntID,invert);
}


bool MechanicDevice::setJointAngles(const std::vector<double> &t_angles)
{
    if(t_angles.size()!=getJointNumbers())
        return false;
    for(uint i = 0; i < t_angles.size(); i++){
        setJointAngle(i, t_angles[i]);
    }
    return true;
}

bool MechanicDevice::setJointAngle(int jntID, float jntAngle)
{
    if(0<= jntID && jntID < AxisValues.getValues().size()){
        m_kinematicModel.setJointAngle(jntID, jntAngle);
        updateAxisValues();
        return true;
    }
    return false;
}

const Base::Placement MechanicDevice::getJointTransformation(const int jntID) const
{
    Base::Placement result;
    auto kin_JntPose = m_kinematicModel.getJointPose(jntID);
    auto buffer = getOriginPose().toMatrix() * kin_JntPose.toMatrix();
    result.fromMatrix(buffer);
    return result;
}

bool MechanicDevice::isAxisDirInverted(uint j_ID)
{
    return m_kinematicModel.JointDirInverted(j_ID);
}

bool MechanicDevice::setupJointChain(const std::vector<Base::Placement> &poseVec)
{
    return m_kinematicModel.setupJointChain(poseVec);
}

void MechanicDevice::updateAxisValues()
{
    AxisValues.setValues(m_kinematicModel.getJointAngles());
}

const std::vector<double> MechanicDevice::getAxisRuningSpeed() const
{
    auto speedLimits = AxisRatedSpeed.getValues();
    auto limitRatio = AxisSpeedRatio.getValue();
    std::vector<double> result = std::vector<double>(speedLimits.size(), 0.0);
    for(auto i = 0; i<speedLimits.size(); i++){
        result[i] = speedLimits[i] * limitRatio;
    }
    return result;
}

const int MechanicDevice::getJointNumbers() const
{
    return m_kinematicModel.getJointNumbers();
}

const float MechanicDevice::getJointAngle(const int JointID) const
{
    return m_kinematicModel.getJointAngle(JointID);
}

const std::vector<double> MechanicDevice::getJointAngles() const
{
    return m_kinematicModel.getJointAngles();
}

void MechanicDevice::setAttachable(bool attachable)
{
    MoveWithAttachedBody.setValue(attachable);
}

void MechanicDevice::setBaseToSelectedFaceCenter()
{
//    auto targetEntity = dynamic_cast<Part::Feature *>(LinkedFaceFeature.getValue());
//    if (targetEntity != nullptr)
//        targetEntity->setAttachedObj(this);
//    auto newCenter = CAD_Utility::calculateLinkedFaceCenter(LinkedFaceFeature);
    //    Pose_Ref2Base.setValue(Base::Placement(newCenter.toMatrix() /** offset.toMatrix()*/));
}





