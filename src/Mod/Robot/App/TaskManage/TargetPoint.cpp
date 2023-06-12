
#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include "TargetPoint.h"

#ifndef M_PI
    #define M_PI    3.14159265358979323846 /* pi */
#endif

#ifndef M_PI_2
    #define M_PI_2  1.57079632679489661923 /* pi/2 */
#endif

using namespace Robot;
using namespace Base;

TYPESYSTEM_SOURCE(Robot::TargetPoint , Base::Persistence);

TargetPoint::TargetPoint()
{
}

TargetPoint::TargetPoint(const TargetPoint &pnt)
{
    operator =(pnt);
}

TargetPoint &TargetPoint::operator=(const TargetPoint &pnt)
{
    m_PoseID = pnt.m_PoseID;
    m_CartPose = pnt.m_CartPose;
    m_MechPose = pnt.m_MechPose;
    updateWaypointName();
}

bool TargetPoint::operator==(const TargetPoint &rhs)
{
    return this->getPointHashID() == rhs.getPointHashID();
}

TargetPoint::~TargetPoint()
{
}

unsigned int TargetPoint::getMemSize (void) const
{
	return 0;
}

void TargetPoint::Save (Writer &writer) const
{
    writer.Stream() << writer.ind();
    writer.Stream() << "<Waypoint ID=\""<<std::to_string(m_PoseID)<<"\">";
//    m_PoseData.Save(writer);
    writer.Stream() << "</Waypoint>"<<std::endl;
}

void TargetPoint::Restore(XMLReader &reader)
{
//    // read my Element
    reader.readElement("Waypoint");
    m_PoseID = reader.getAttributeAsUnsigned("ID");
//    m_PoseData.Restore(reader);
    reader.readEndElement("Waypoint");
}

bool TargetPoint::isValid() const
{
//    return m_PoseData.isValid();
    return true;
}

#include <iomanip>
#include <iostream>

void TargetPoint::updateWaypointName()
{
    auto to_str = [](double val, int precision){
        std::stringstream ss;
        ss<<std::setiosflags(std::ios::fixed)<<std::setprecision(precision)<<val;
        return ss.str();
    };

    std::string wpName;
//    if(this->m_PoseData.Pose_Rbt1.CordInfo.first == CordType::ACS){
//        auto jntVals = this->m_PoseData.Pose_Rbt1.PoseData;
//        std::string poseStr = "[";
//        for(int i=0; i<jntVals.size();i++){
//            poseStr += "J"+std::to_string(i)+":"+ to_str(jntVals[i],1)+", ";
//        }
//        poseStr+="]";
//        wpName += poseStr;
//    }

//    else{
//        auto pose = this->m_PoseData.Pose_Rbt1.PoseData;
//        auto poseStr =  "[x:"+to_str(pose[0],1)+", "
//                         "y:"+to_str(pose[1],1)+", "
//                         "z:"+to_str(pose[2],1)+"| "
//                         "R:"+to_str(pose[5],1)+", "
//                         "P:"+to_str(pose[4],1)+", "
//                         "Y:"+to_str(pose[3],1)+"]";
//        wpName += poseStr;
//    }
    this->setWP_Name(wpName);
}

void RobotPose::setPoseData(const Placement &t_Pose)
{
    FlanPose = t_Pose;
    PoseData[0] = t_Pose.getPosition().x;
    PoseData[1] = t_Pose.getPosition().y;
    PoseData[2] = t_Pose.getPosition().z;
    double y,p,r;
    t_Pose.getRotation().getYawPitchRoll(y,p,r);
    PoseData[3] = y;
    PoseData[4] = p;
    PoseData[5] = r;
}

//void RobotPose::setPoseData(const CompPose &t_Pose)
//{
////    FlanPose = t_Pose.first;
////    PoseData[0] = t_Pose.first.getPosition().x;
////    PoseData[1] = t_Pose.first.getPosition().y;
////    PoseData[2] = t_Pose.first.getPosition().z;
////    double y,p,r;
////    t_Pose.first.getRotation().getYawPitchRoll(y,p,r);
////    PoseData[3] = y;
////    PoseData[4] = p;
////    PoseData[5] = r;
////    PoseData[6] = t_Pose.second.first;
////    PoseData[7] = t_Pose.second.second;
//}

void RobotPose::setPoseData(const std::vector<double> &t_JntVals)
{
    if(t_JntVals.size()>8)
        return;
    PoseData.clear();
    std::copy(t_JntVals.begin(), t_JntVals.end(), PoseData.begin());
    CordInfo.first = CordType::ACS;
}

void RobotPose::Save(Writer &writer) const
{
    writer.Stream()<<"CordType=\""<<(int)CordInfo.first<<"\" ";
    writer.Stream()<<"CordID=\""<<(int)CordInfo.second<<"\" ";
    writer.Stream()<<"CfgID=\""<<ConfigID<<"\" ";
    int i = 0;
    for(int i = 0; i<8; i++){
        writer.Stream()<<"P"<<std::to_string(i)<<"=\""<<std::to_string(PoseData[i])<<"\" ";
    }

    writer.Stream() << "Px=\""          <<  FlanPose.getPosition().x  << "\" "
                    << "Py=\""          <<  FlanPose.getPosition().y  << "\" "
                    << "Pz=\""          <<  FlanPose.getPosition().z  << "\" "
                    << "Q0=\""          <<  FlanPose.getRotation()[0] << "\" "
                    << "Q1=\""          <<  FlanPose.getRotation()[1] << "\" "
                    << "Q2=\""          <<  FlanPose.getRotation()[2] << "\" "
                    << "Q3=\""          <<  FlanPose.getRotation()[3] << "\" "
                    << std::endl;
}

void RobotPose::Restore(XMLReader &reader)
{
    CordInfo.first = (CordType)reader.getAttributeAsInteger("CordType");
    CordInfo.second = reader.getAttributeAsInteger("CordID");
    ConfigID = reader.getAttributeAsUnsigned("CfgID");
    for(int i = 0; i<8;i++){
        auto t_name = std::string("P")+std::to_string(i);
        PoseData[i] = reader.getAttributeAsFloat(t_name.c_str());
    }
    FlanPose = Base::Placement(Base::Vector3d(reader.getAttributeAsFloat("Px"),
                                              reader.getAttributeAsFloat("Py"),
                                              reader.getAttributeAsFloat("Pz")),
                               Base::Rotation(reader.getAttributeAsFloat("Q0"),
                                              reader.getAttributeAsFloat("Q1"),
                                              reader.getAttributeAsFloat("Q2"),
                                              reader.getAttributeAsFloat("Q3")));
}

const Placement RobotPose::getCartPose() const
{
    if(CordInfo.first == CordType::ACS)
        return Placement();
    Base::Placement c_Pose;
    Base::Rotation c_Rot;
    c_Pose.setPosition(Base::Vector3d(PoseData[0],PoseData[1],PoseData[2]));
    c_Rot.setYawPitchRoll(PoseData[3],PoseData[4],PoseData[5]);
    c_Pose.setRotation(c_Rot);
    return c_Pose;
}
