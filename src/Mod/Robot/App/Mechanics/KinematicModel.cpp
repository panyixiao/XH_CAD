#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "KinematicModel.h"
#include "RobotAlgos.h"

#ifndef M_PI
    #define M_PI    3.14159265358979323846 /* pi */
#endif

#ifndef M_PI_2
    #define M_PI_2  1.57079632679489661923 /* pi/2 */
#endif

using namespace Robot;
using namespace Base;
using namespace KDL;

TYPESYSTEM_SOURCE(Robot::KinematicModel , Base::Persistence);

KinematicModel::KinematicModel()
{
}

KinematicModel::~KinematicModel()
{
}


bool KinematicModel::setupJointChain(const std::vector<Placement> &axisPoses)
{
    Chain temp;
    for(int i=1 ; i<=axisPoses.size() ;i++){
        auto t_pose = axisPoses[i];
        Joint::JointType t_Type;
        if(axisPoses.size() == 6){
            if(i == 1 || i == 4){
                t_Type = Joint::RotZ;
            }else{
                t_Type = Joint::RotX;
            }
        }
        temp.addSegment(Segment(Joint(t_Type),
                        Frame(KDL::Rotation::Quaternion(t_pose.getRotation()[0],
                                                        t_pose.getRotation()[1],
                                                        t_pose.getRotation()[2],
                                                        t_pose.getRotation()[3]),
                              KDL::Vector(t_pose.getPosition().x,
                                          t_pose.getPosition().y,
                                          t_pose.getPosition().z))));
    }
    // for now and testing
    m_KDLchain = temp;
    updateJointFrames();
    return true;
}

const double KinematicModel::getMaxAngle(int Axis) const
{
    return m_JointUpperLimits(Axis) * (180.0/M_PI);
}

const double KinematicModel::getMinAngle(int Axis) const
{
    return m_JointLowerLimits(Axis) * (180.0/M_PI);
}

const std::vector<double> KinematicModel::getJointAngles() const
{
    std::vector<double> jntVal_deg;
    for(int i = 0; i<m_JointValues.rows();i++){
        double radVal = m_JointRotDir(i) * m_JointValues(i);
        jntVal_deg.push_back(radVal * 180 / M_PI);
    }
    return jntVal_deg;
}

const std::vector<double> KinematicModel::getJointMaxAngles() const
{
    std::vector<double> result;
    for(int i = 0; i<m_JointUpperLimits.rows(); i++){
        result.push_back(m_JointUpperLimits(i)*180/M_PI);
    }
    return result;
}

const std::vector<double> KinematicModel::getJointMinAngles() const
{
    std::vector<double> result;
    for(int i = 0; i<m_JointLowerLimits.rows(); i++){
        result.push_back(m_JointLowerLimits(i)*180/M_PI);
    }
    return result;
}


bool KinematicModel::readURDFfiles(const char *FileName)
{
    m_RobotModel = urdf::parseURDFFile(std::string(FileName));
    if(m_RobotModel==nullptr)
        return false;
    if(updateKDLChain())
        setKinematicModelConfig(ConfigType::NON);
    return false;
}

bool KinematicModel::updateKDLChain()
{
    if(!kdl_parser::treeFromUrdfModel(*m_RobotModel, m_KDLtree))
        return false;
    std::vector<urdf::LinkSharedPtr> t_links;
    m_RobotModel->getLinks(t_links);
    if(t_links.empty())
        return false;
    // Set Kinematic Chain
    jointNumber = t_links.size();
    std::vector<urdf::LinkSharedPtr> t_validLinks;
    uint s_id, e_id;

    bool baseOnBack = false;
    // Dealing With base_link at begin or rear
    if(t_links.front()->parent_joint == nullptr){
        s_id = 0;
        e_id = t_links.size() - 1;
        t_validLinks.insert(t_validLinks.end(),t_links.begin()+1,t_links.end());
    }
    else if(t_links.back()->parent_joint == nullptr){
        s_id = t_links.size() - 1;
        e_id = t_links.size() - 2;
        t_validLinks.insert(t_validLinks.end(),t_links.begin(),t_links.end()-1);
        jointNumber--;
        baseOnBack = true;
    }
    m_linkNames.clear();
    m_linkNames.push_back(t_links.at(s_id)->name);
    int link1_id, last_id;
    if(baseOnBack){
        link1_id = 0;
        last_id = t_links.size()-1;
    }else{
        link1_id = 1;
        last_id = t_links.size();
    }
    for(auto i = link1_id; i<last_id; i++){
        m_linkNames.push_back(t_links.at(i)->name);
    }

    // Constructing Tree
    auto rootName = t_links.at(s_id)->name;
    auto tipName = t_links.at(e_id)->name;
    if(!m_KDLtree.getChain(rootName, tipName, m_KDLchain)){
        Base::Console().Error("KinematicModel: Unable to Generate Kinematic Chain From KDLTree!\n");
        return false;
    }
    m_FkSolverPtr = new KDL::ChainFkSolverPos_recursive(m_KDLchain);
    m_IkVelSolverPtr = new KDL::ChainIkSolverVel_pinv(m_KDLchain);

    jointNumber = t_validLinks.size();
    // Set Joint Upper/Lower Limits
    m_JointValues = KDL::JntArray(jointNumber);
    m_JointLowerLimits = KDL::JntArray(jointNumber);
    m_JointUpperLimits = KDL::JntArray(jointNumber);
    m_JointSpeedLimits = KDL::JntArray(jointNumber);
    m_JointRotDir = KDL::JntArray(jointNumber);
    jointFrameVec.clear();
    jointFrameVec.resize(jointNumber);
    for(uint i = 0; i<t_validLinks.size(); i++){
        m_JointValues(i) = 0.0;
        m_JointRotDir(i) = 1.0;
        m_JointLowerLimits(i) = t_validLinks[i]->parent_joint->limits->lower;
        m_JointUpperLimits(i)  = t_validLinks[i]->parent_joint->limits->upper;
        m_JointSpeedLimits(i)  = t_validLinks[i]->parent_joint->limits->velocity;
    }
    updateJointFrames();
    return true;
}

void KinematicModel::setIKsolverType(const IK_SolverType t_Type)
{
    m_solverType = t_Type;
}

unsigned int KinematicModel::getMemSize (void) const
{
	return 0;
}

void KinematicModel::Save (Writer &writer) const
{
    writer.Stream() << writer.ind();
    writer.Stream() << "<Segments Count=\""<<jointNumber-1<<"\">";
    saveChain(writer);
    writer.Stream() <<  "</Segments>"<<std::endl;
    writer.Stream() << "<MechanicType Type= \""<<(int)m_MechanicType<<"\"/>"<<std::endl;
    writer.Stream() << "<ConfigType Type= \""<<(int)m_ConfigType<<"\"/>"<<std::endl;
}

void KinematicModel::Restore(XMLReader &reader)
{
    reader.readElement("Segments");
    if(reader.hasAttribute("Count")){
        jointNumber = reader.getAttributeAsInteger("Count")+1;

        m_JointValues = KDL::JntArray(jointNumber);

        m_JointLowerLimits = KDL::JntArray(jointNumber);
        m_JointUpperLimits = KDL::JntArray(jointNumber);

        m_JointSpeedLimits = KDL::JntArray(jointNumber);
        m_JointRotDir = KDL::JntArray(jointNumber);

        jointFrameVec.resize(jointNumber);

        for(unsigned int i=0;i<jointNumber;i++){
            // read my Element
            reader.readElement("Axis");
            // get the value of the placement
            auto axisOrigin =  Base::Placement(Base::Vector3d(reader.getAttributeAsFloat("Px"),
                                                              reader.getAttributeAsFloat("Py"),
                                                              reader.getAttributeAsFloat("Pz")),
                                               Base::Rotation(reader.getAttributeAsFloat("Q0"),
                                                              reader.getAttributeAsFloat("Q1"),
                                                              reader.getAttributeAsFloat("Q2"),
                                                              reader.getAttributeAsFloat("Q3")));
            m_KDLchain.addSegment(Segment(Joint(Joint::RotZ),toFrame(axisOrigin)));
            m_JointRotDir(i) = reader.getAttributeAsInteger("rotDir");
            // read the axis constraints
            m_JointLowerLimits(i)  = reader.getAttributeAsFloat("minAngle")* (M_PI/180);
            m_JointUpperLimits(i)  = reader.getAttributeAsFloat("maxAngle")* (M_PI/180);
            m_JointSpeedLimits(i) = reader.getAttributeAsFloat("AxisVelocity");
            m_JointValues(i) = reader.getAttributeAsFloat("JointPos");
        }
        m_FkSolverPtr = new KDL::ChainFkSolverPos_recursive(m_KDLchain);
        m_IkVelSolverPtr = new KDL::ChainIkSolverVel_pinv(m_KDLchain);
    }
    reader.readElement("MechanicType");
    if(reader.hasAttribute("Type"))
        m_MechanicType = (MechanicType)reader.getAttributeAsInteger("Type");

    reader.readElement("ConfigType");
    if(reader.hasAttribute("Type"))
        m_ConfigType = (ConfigType)reader.getAttributeAsInteger("Type");
    setKinematicModelConfig(m_ConfigType);
    reader.readEndElement("Segments");
}

const bool KinematicModel::initialized() const
{
    return m_tracik_solver!=nullptr &&
           m_kdl_solver!=nullptr &&
            m_KDLtree.getNrOfJoints()!=0;
}

void KinematicModel::setMechanicType(const MechanicType &t_Type)
{
    m_MechanicType = t_Type;
}

bool KinematicModel::setTo(const Placement &t_Pose)
{
    if(!initialized())
        return false;

    Frame F_dest = Frame(KDL::Rotation::Quaternion(t_Pose.getRotation()[0],
                                                   t_Pose.getRotation()[1],
                                                   t_Pose.getRotation()[2],
                                                   t_Pose.getRotation()[3]),
                         KDL::Vector(t_Pose.getPosition()[0]/trans_ratio,
                                     t_Pose.getPosition()[1]/trans_ratio,
                                     t_Pose.getPosition()[2]/trans_ratio));

	//Creation of jntarrays:
    JntArray ik_solution(m_KDLchain.getNrOfJoints());
    bool success = false;
    KDL::JntArray initState;
    if(m_ConfigType!=ConfigType::NON)
        initState = KDL::JntArray(jointNumber);
    else
        initState = m_JointValues;

    switch (m_solverType) {
    case IK_SolverType::TRAC_IK:
        success = m_tracik_solver->CartToJnt(initState,F_dest,ik_solution) > 0;
        break;
    case IK_SolverType::KDL_IK:
        success = m_kdl_solver->CartToJnt(initState,F_dest,ik_solution) > 0;
        break;
    default:
        break;
    }

    if(success){
        m_JointValues = ik_solution;
    }

    return success;
}

bool KinematicModel::flipJointDir(uint j_ID, bool invert)
{
    if(j_ID >= m_JointRotDir.rows())
        return false;
    if(invert)
        m_JointRotDir(j_ID)=-1;
    else
        m_JointRotDir(j_ID)=1;
    return true;
}

bool KinematicModel::JointDirInverted(uint j_ID)
{
    if(j_ID < m_JointRotDir.rows())
        return m_JointRotDir(j_ID) == -1;
    return false;
}

const Placement KinematicModel::getTcp(void) const
{
    double x,y,z,w;
    m_TCPframe.M.GetQuaternion(x,y,z,w);
    return Base::Placement(Base::Vector3d(trans_ratio * m_TCPframe.p[0],
                                          trans_ratio * m_TCPframe.p[1],
                                          trans_ratio * m_TCPframe.p[2]),
            Base::Rotation(x,y,z,w));
}

void KinematicModel::setKinematicModelConfig(const ConfigType &t_Type)
{
    if(m_MechanicType != MechanicType::M_Robot)
        m_ConfigType = ConfigType::NON;
    else
        m_ConfigType = t_Type;
    updateJointLimitsByConfigType();
    updateIkSolvers();
}

const ConfigType &KinematicModel::getConfigType() const
{
    return m_ConfigType;
}

const Placement KinematicModel::getJointPose(int jntID) const
{
    if(jointFrameVec.empty())
        return Base::Placement();
    auto c_Pose = jointFrameVec[jntID];
    double qx,qy,qz,qw;
    c_Pose.M.GetQuaternion(qx,qy,qz,qw);
    return Base::Placement(Base::Vector3d(trans_ratio*c_Pose.p[0],
                                          trans_ratio*c_Pose.p[1],
                                          trans_ratio*c_Pose.p[2]),
                           Base::Rotation(qx,qy,qz,qw));
}

void KinematicModel::updateJointFrames()
{
    if(m_FkSolverPtr == nullptr)
        return;
     m_FkSolverPtr->JntToCart(m_JointValues,jointFrameVec);
     if(!jointFrameVec.empty())
        m_TCPframe = jointFrameVec.back();
}

void KinematicModel::updateIkSolvers()
{
    if(m_kdl_solver!=nullptr)
        delete m_kdl_solver;
    m_kdl_solver = new ChainIkSolverPos_NR_JL(m_KDLchain,
                                              m_JointLowerLimits_Configured,
                                              m_JointUpperLimits_Configured,
                                              *m_FkSolverPtr,
                                              *m_IkVelSolverPtr,
                                              100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    if(m_tracik_solver!=nullptr)
        delete m_tracik_solver;
    m_tracik_solver = new TRAC_IK::TRAC_IK(m_KDLchain,
                                           m_JointLowerLimits_Configured,
                                           m_JointUpperLimits_Configured);
}

void KinematicModel::saveChain(Writer &writer) const
{
    if(m_KDLchain.getNrOfJoints() == 0)
        return;
    for(unsigned int i=0;i<jointNumber-1;i++){
        Base::Placement t_AxisOriginPose = toPlacement(m_KDLchain.getSegment(i).getFrameToTip());
        writer.Stream() << writer.ind();
        writer.Stream() << "<Axis "
                        << "Px=\""          <<  t_AxisOriginPose.getPosition().x  << "\" "
                        << "Py=\""          <<  t_AxisOriginPose.getPosition().y  << "\" "
                        << "Pz=\""          <<  t_AxisOriginPose.getPosition().z  << "\" "
                        << "Q0=\""          <<  t_AxisOriginPose.getRotation()[0] << "\" "
                        << "Q1=\""          <<  t_AxisOriginPose.getRotation()[1] << "\" "
                        << "Q2=\""          <<  t_AxisOriginPose.getRotation()[2] << "\" "
                        << "Q3=\""          <<  t_AxisOriginPose.getRotation()[3] << "\" "
                        << "rotDir=\""      <<  m_JointRotDir(i)            << "\" "
                        << "maxAngle=\""    <<  m_JointUpperLimits(i)*(180.0/M_PI)  << "\" "
                        << "minAngle=\""    <<  m_JointLowerLimits(i)*(180.0/M_PI)  << "\" "
                        << "AxisVelocity=\""<<  m_JointSpeedLimits(i)       << "\" "
                        << "JointPos=\""    <<  m_JointValues(i)            << "\"/>"
                        << std::endl;
    }
}

void KinematicModel::updateJointLimitsByConfigType()
{
    m_JointLowerLimits_Configured = m_JointLowerLimits;
    m_JointUpperLimits_Configured = m_JointUpperLimits;
    if(m_MechanicType != MechanicType::M_Robot ||
       m_JointLowerLimits_Configured.rows() < 6||
       m_JointUpperLimits_Configured.rows() < 6)
        return;

//    double initPose[6] = {0.0, 0.0, M_PI/2, M_PI/2, 0.0, 0.0};

//    // TODO: Need to Consider about Joint Initial Position
//    bool flag_Nonflip, flag_RhtSide, flag_UpElbow;
//    switch(m_ConfigType){
//    case ConfigType::NON:
//        break;
//    case ConfigType::NRU:
//        flag_Nonflip = true;
//        flag_RhtSide = true;
//        flag_UpElbow = true;
//        break;
//    case ConfigType::NRD:
//        flag_Nonflip = true;
//        flag_RhtSide = true;
//        flag_UpElbow = false;
//        break;
//    case ConfigType::NLU:
//        flag_Nonflip = true;
//        flag_RhtSide = false;
//        flag_UpElbow = true;
//        break;
//    case ConfigType::NLD:
//        flag_Nonflip = true;
//        flag_RhtSide = false;
//        flag_UpElbow = false;
//        break;
//    case ConfigType::FRU:
//        flag_Nonflip = false;
//        flag_RhtSide = true;
//        flag_UpElbow = true;
//        break;
//    case ConfigType::FRD:
//        flag_Nonflip = false;
//        flag_RhtSide = true;
//        flag_UpElbow = false;
//        break;
//    case ConfigType::FLU:
//        flag_Nonflip = false;
//        flag_RhtSide = false;
//        flag_UpElbow = true;
//        break;
//    case ConfigType::FLD:
//        flag_Nonflip = false;
//        flag_RhtSide = false;
//        flag_UpElbow = false;
//        break;
//    }

//    if(flag_Nonflip){
//        m_JointLowerLimits_Configured.data(4) = 0.1 - initPose[4];
//        m_JointUpperLimits_Configured.data(4) = std::min(M_PI - initPose[4], m_JointUpperLimits.data(4));
//    }else{
//        m_JointLowerLimits_Configured.data(4) = std::max(-M_PI + initPose[4], m_JointLowerLimits.data(4));
//        m_JointUpperLimits_Configured.data(4) = 0.1 + initPose[4];
//    }

//    if(flag_RhtSide){
//        m_JointLowerLimits_Configured.data(3) = 0 - initPose[3];
//        m_JointUpperLimits_Configured.data(3) = std::min(M_PI - initPose[3], m_JointUpperLimits.data(3));
//    }else{
//        m_JointLowerLimits_Configured.data(3) = std::max(-M_PI + initPose[3], m_JointLowerLimits.data(3));
//        m_JointUpperLimits_Configured.data(3) = 0 + initPose[3];
//    }

//    if(flag_UpElbow){
//        m_JointLowerLimits_Configured.data(2) = 0 - initPose[2];
//        m_JointUpperLimits_Configured.data(2) = std::min(M_PI - initPose[2], m_JointUpperLimits.data(2));
//    }else{
//        m_JointLowerLimits_Configured.data(2) = std::max(-M_PI + initPose[2], m_JointLowerLimits.data(2));
//        m_JointUpperLimits_Configured.data(2) = 0 + initPose[2];
//    }
}

bool KinematicModel::setJointAngle(int j_ID,double Value)
{
    if(j_ID < 0 || j_ID > m_JointValues.rows())
        return false;
    m_JointValues(j_ID) = m_JointRotDir(j_ID) * Value * (M_PI/180); // degree to radiants
    updateJointFrames();
    return true;
}

bool KinematicModel::setJointAngles(const std::vector<double> &jntAngles)
{
    if(jntAngles.size()!=m_JointValues.rows())
        return false;
    for(int i = 0; i<m_JointValues.rows(); i++){
        m_JointValues(i) = m_JointRotDir(i)*jntAngles[i]*(M_PI/180);
    }
    updateJointFrames();
    return true;
}

bool KinematicModel::setJointMaxLimits(const std::vector<double> &MaxAngles)
{
    if(MaxAngles.size() != m_JointUpperLimits.rows())
        return false;
    for(int jntID = 0; jntID < MaxAngles.size(); jntID++){
        m_JointUpperLimits(jntID) = MaxAngles[jntID] * (M_PI/180);
    }
    return true;
}

bool KinematicModel::setJointMinLimits(const std::vector<double> &MinAngles)
{
    if(MinAngles.size() != m_JointLowerLimits.rows())
        return false;
    for(int jntID = 0; jntID < MinAngles.size(); jntID++){
        m_JointLowerLimits(jntID) = MinAngles[jntID] * (M_PI/180);
    }
    return true;
}

const std::vector<string> KinematicModel::getJointNames() const
{
    std::vector<string> result;
    for(auto i = 0; i<m_KDLchain.getNrOfSegments(); i++){
        auto t_seg = m_KDLchain.getSegment(i);
        result.push_back(t_seg.getJoint().getName());
    }
    return result;
}

const std::vector<string> &KinematicModel::getLinkNames() const
{
    return m_linkNames;
}

void KinematicModel::setJointSpeedLimit(const uint t_ID, double degPersec)
{
    if(t_ID < getJointNumbers())
       m_JointSpeedLimits.data(t_ID) = degPersec*M_PI/180;
}

const std::vector<double> KinematicModel::getJointSpeedLimits() const
{
    auto jntNbr = getJointNumbers();
    std::vector<double> speedLimits = std::vector<double>(jntNbr,0.0);
    for(uint i = 0; i < jntNbr ; i++){
        speedLimits[i] = m_JointSpeedLimits.data(i)*180/M_PI;
    }
    return speedLimits;
}

const double KinematicModel::getJointAngle(int Axis) const
{
    return m_JointRotDir(Axis) * (m_JointValues(Axis)/(M_PI/180)); // radian to degree
}

