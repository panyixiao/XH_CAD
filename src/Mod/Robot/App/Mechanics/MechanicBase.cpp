
#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "MechanicBase.h"
#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicBase, App::GeoFeature)

MechanicBase::MechanicBase()
:flag_updateJntVals(false)
{
    ADD_PROPERTY_TYPE(FilePath_URDF,(0),"Files",Prop_None,"Identify the filePath of urdf file which contains definition about mechanic model");
    ADD_PROPERTY_TYPE(FilePath_Calibration,(0),"Files",Prop_None,("Identify the filePath of calibration file"));

    ADD_PROPERTY_TYPE(Activated,(false),"Property",Prop_None,"Is Mechanic activated");
    ADD_PROPERTY_TYPE(Visiable,(true),"Property",Prop_None,"Is Mechanic Visiable in world");
    ADD_PROPERTY_TYPE(isDriven, (false),"Property", Prop_Hidden,"Identify is Target is driven or manipulated");
    ADD_PROPERTY_TYPE(isEditing,(0),"Property",Prop_None, "Identify is Target is Editing");
    ADD_PROPERTY_TYPE(DeviceType,(0),"Property",Prop_None, "Current Teach Coord Index");

    ADD_PROPERTY_TYPE(Pose_Referece,(Base::Placement()),"Kinematics",Prop_None,"The Placement of Reference");
    ADD_PROPERTY_TYPE(Trans_Ref2Base,(Base::Placement()),"Kinematics",Prop_None,"Transformation from Reference Origin to Base");
    ADD_PROPERTY_TYPE(Pose_Tip,(Base::Placement()),"Kinematics",Prop_None,"The pose of current active Tcp in WorldFrame");

    ADD_PROPERTY_TYPE(HomePose ,(0.0), "Kinematics",Prop_None,"Axis position for home");
    ADD_PROPERTY_TYPE(AxisValues, (0.0), "Kinematics", Prop_None, "Axis angle of the robot joints");

    ADD_PROPERTY_TYPE(UpperLimits_Real, (0.0), "Kinematics", Prop_None, "Real Upper Limits");
    ADD_PROPERTY_TYPE(UpperLimits_Soft, (0.0), "Kinematics", Prop_None, "Soft Upper Limits");
    ADD_PROPERTY_TYPE(LowerLimits_Real, (0.0), "Kinematics", Prop_None, "Real Lower Limits");
    ADD_PROPERTY_TYPE(LowerLimits_Soft, (0.0), "Kinematics", Prop_None, "Soft Lower Limits");
    ADD_PROPERTY_TYPE(AxisRatedSpeed, (0.0), "Kinematics", Prop_None, "Max Joint Speed of each Axis");
    ADD_PROPERTY_TYPE(AxisSpeedRatio, (1.0), "Kinematics", Prop_None, "Joint Speed Ratio for All Axis Angles");

    Placement.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(LinkedEdgeFeature, (0));
    LinkedEdgeFeature.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(LinkedFaceFeature, (0));
    LinkedFaceFeature.setStatus(App::Property::Status::Hidden, true);
//    ADD_PROPERTY(InteractiveDraggerOn,(false));
//    InteractiveDraggerOn.setStatus(App::Property::Status::Hidden, true);

    m_kinematicModel.setIKsolverType(IK_SolverType::TRAC_IK);
}

MechanicBase::~MechanicBase()
{
}

DocumentObjectExecReturn *MechanicBase::execute()
{
    return StdReturn;
}

DocumentObjectExecReturn *MechanicBase::recompute()
{
    m_kinematicModel.updateJointFrames();
    return this->execute();
}

short MechanicBase::mustExecute(void) const
{
    return 0;
}

PyObject *MechanicBase::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void MechanicBase::onChanged(const Property* prop)
{
   if(prop == &FilePath_URDF){
       if(m_kinematicModel.readURDFfiles(FilePath_URDF.getValue())){
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
            Pose_Tip.setValue(getCurrentTipPose());
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

void MechanicBase::onDocumentRestored()
{
    App::GeoFeature::onDocumentRestored();
    AxisValues.setValues(AxisValues.getValues());
}


const Base::Placement MechanicBase::getCurrentTipPose() const
{
    Base::Placement c_TipPose;
    if(!m_kinematicModel.initialized())
        return c_TipPose;
    Base::Placement abs_TcpPose = m_kinematicModel.getTcp();
    c_TipPose= Base::Placement(getOriginPose().toMatrix() * abs_TcpPose.toMatrix());
    return c_TipPose;
//    Base::Placement result, tool_transform;
//    tool_transform = getToolTipTranslation();
//    result = Base::Placement(getCurrentFlanPose(ref_Origin,
//                                                origin_Pose).toMatrix() *
//                             tool_transform.toMatrix());

//    return result;
}


//const Base::Placement MechanicBase::getTeachDraggerPose() const
//{
//    return getCurrentTipPose();
//}

bool MechanicBase::setAxisHomePose()
{
    auto c_jntVals = m_kinematicModel.getJointAngles();
    if(c_jntVals.size() != HomePose.getValues().size())
        return false;
    HomePose.setValues(c_jntVals);
    return true;
}

bool MechanicBase::resAxisHomePose()
{
    if(m_kinematicModel.setJointAngles(HomePose.getValues())){
        updateAxisValues();
        return true;
    }
    return false;
}

float MechanicBase::getJointMaxAngle(const uint JointID) const
{
    return m_kinematicModel.getMaxAngle(JointID);
}

float MechanicBase::getJointMinAngle(const uint JointID) const
{
    return m_kinematicModel.getMinAngle(JointID);
}

const std::vector<double> MechanicBase::getJointMaxAngles() const
{
    return m_kinematicModel.getJointMaxAngles();
}

const std::vector<double> MechanicBase::getJointMinAngles() const
{
    return m_kinematicModel.getJointMinAngles();
}


void MechanicBase::updateJointLimits()
{
    m_kinematicModel.setJointMaxLimits(UpperLimits_Real.getValues());
    m_kinematicModel.setJointMinLimits(LowerLimits_Real.getValues());
}

void MechanicBase::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    m_kinematicModel.Save(writer);
}

void MechanicBase::Restore(Base::XMLReader &reader)
{
    App::GeoFeature::Restore(reader);
    flag_updateJntVals = true;
    m_kinematicModel.Restore(reader);
    flag_updateJntVals = false;
}

const std::vector<string> MechanicBase::getAxisNames() const
{
    auto b_Name = std::string(getNameInDocument());
    auto names = m_kinematicModel.getJointNames();
    std::vector<string> result;
    for(auto j_Name : names){
        result.push_back(b_Name+std::string("_")+j_Name);
    }
    return result;
}

KinematicModel &MechanicBase::getKinematicModelRef()
{
    return m_kinematicModel;
}

void MechanicBase::setAxisOriginPose(const uint axisID, const Base::Placement &t_Pose)
{
}

const Base::Placement MechanicBase::getAxisOriginPose(const uint t_AxisID)
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

const Base::Placement MechanicBase::getOriginPose() const
{
    return Pose_Referece.getValue()*Trans_Ref2Base.getValue();
}

bool MechanicBase::flipAxisDirection(uint jntID, bool invert)
{
    return m_kinematicModel.flipJointDir(jntID,invert);
}


bool MechanicBase::setJointAngles(const std::vector<double> &t_angles)
{
    if(t_angles.size()!=getJointNumbers())
        return false;
    for(uint i = 0; i < t_angles.size(); i++){
        setJointAngle(i, t_angles[i]);
    }
    return true;
}

bool MechanicBase::setJointAngle(const size_t jntID, float jntAngle)
{
    if(0<= jntID && jntID < AxisValues.getValues().size()){
        m_kinematicModel.setJointAngle(jntID, jntAngle);
        updateAxisValues();
        return true;
    }
    return false;
}

const Base::Placement MechanicBase::getJointTransformation(const int jntID) const
{
    Base::Placement result;
    auto kin_JntPose = m_kinematicModel.getJointPose(jntID);
    auto buffer = getOriginPose().toMatrix() * kin_JntPose.toMatrix();
    result.fromMatrix(buffer);
    return result;
}

bool MechanicBase::isAxisDirInverted(uint j_ID)
{
    return m_kinematicModel.JointDirInverted(j_ID);
}

bool MechanicBase::setupJointChain(const std::vector<Base::Placement> &poseVec)
{
    return m_kinematicModel.setupJointChain(poseVec);
}

void MechanicBase::updateAxisValues()
{
    AxisValues.setValues(m_kinematicModel.getJointAngles());
}

const std::vector<double> MechanicBase::getAxisRuningSpeed() const
{
    auto speedLimits = AxisRatedSpeed.getValues();
    auto limitRatio = AxisSpeedRatio.getValue();
    std::vector<double> result = std::vector<double>(speedLimits.size(), 0.0);
    for(size_t i = 0; i<speedLimits.size(); i++){
        result[i] = speedLimits[i] * limitRatio;
    }
    return result;
}

size_t MechanicBase::getJointNumbers() const
{
    return m_kinematicModel.getJointNumbers();
}

float MechanicBase::getJointAngle(const int JointID) const
{
    return m_kinematicModel.getJointAngle(JointID);
}

const std::vector<double> MechanicBase::getJointAngles() const
{
    return m_kinematicModel.getJointAngles();
}





