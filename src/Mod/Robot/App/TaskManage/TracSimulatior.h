// Created by Yixiao 2022-05-21

#ifndef _TracSimulator_h_
#define _TracSimulator_h_

#include <App/Document.h>
#include <Base/Vector3D.h>
#include <Base/Placement.h>
#include <string>
#include <memory>

#include "Mod/Robot/App/Mechanics/RobotAlgos.h"
#include "Mod/Robot/App/Trac/RobotWaypoint.h"
#include "Mod/Robot/App/Mechanics/MechanicGroup.h"
#include "Mod/Robot/App/Trac/RobotProgram.h"

#include "Mod/Robot/App/kdl_cp/path_jointspace.h"
#include "Mod/Robot/App/kdl_cp/trajectory.hpp"
#include "Mod/Robot/App/kdl_cp/trajectory_composite.hpp"
#include "Mod/Robot/App/kdl_cp/trajectory_jointspace.h"


#include "Mod/Robot/App/kdl_cp/chain.hpp"
#include "Mod/Robot/App/kdl_cp/frames_io.hpp"

#include "Mod/Robot/App/kdl_cp/chain.hpp"
#include "Mod/Robot/App/kdl_cp/rotational_interpolation_sa.hpp"
#include "Mod/Robot/App/kdl_cp/path_roundedcomposite.hpp"
#include "Mod/Robot/App/kdl_cp/path_line.hpp"
#include "Mod/Robot/App/kdl_cp/path_point.hpp"
#include "Mod/Robot/App/kdl_cp/path_circle.hpp"
#include "Mod/Robot/App/kdl_cp/velocityprofile_trap.hpp"

#include "Mod/Robot/App/kdl_cp/trajectory_segment.hpp"
#include "Mod/Robot/App/kdl_cp/trajectory_composite.hpp"

#include "Mod/Robot/App/kdl_cp/utilities/error.h"

#define DefaultAcc 100.0 // mm/s2

using SimTrac_sptr = std::shared_ptr<KDL::Trajectory_Composite>;
using JntTrac_sptr = std::shared_ptr<KDL::Trajectory_JointSpace>;
using RobotProg_sptr = std::shared_ptr<Robot::RobotProgram>;

namespace Robot
{

class SimFrame{
public:
    SimFrame(){}
    SimFrame(const std::vector<double>& t_Pose){
        C_Type = CordType::ACS;
        frame_Jnts.resize(t_Pose.size());
        for(int i = 0; i<t_Pose.size(); i++){
            frame_Jnts.data(i) = t_Pose[i];
        }
    }

    SimFrame(const RobotPose& t_Pose){
        C_Type = t_Pose.CordInfo.first;
        if(C_Type == CordType::ACS){
            frame_Jnts.resize(8);
            for(int i = 0; i<8; i++){
                frame_Jnts.data(i) = t_Pose.PoseData[i];
            }
        }
        else{
            frame_Cart = toFrame(t_Pose.getCartPose());
        }
    }

    SimFrame& operator = (SimFrame& rhs){
        C_Type = rhs.C_Type;
        frame_Cart = rhs.frame_Cart;
        frame_Jnts = rhs.frame_Jnts;
    }

public:
    CordType C_Type;
    KDL::Frame frame_Cart;
    KDL::JntArray frame_Jnts;
};

class GroupFrame{
public:
    GroupFrame(const GroupPose& t_Pose){
        if(t_Pose.Pose_Rbt1.isValid())
            gp1_Frame = std::make_shared<SimFrame>(t_Pose.Pose_Rbt1);
        if(t_Pose.Pose_Rbt2.isValid())
            gp2_Frame = std::make_shared<SimFrame>(t_Pose.Pose_Rbt2);
        ext_Frame = std::make_shared<SimFrame>(t_Pose.ExtVals);
    }
    GroupFrame& operator =(GroupFrame& rhs){
        gp1_Frame = rhs.gp1_Frame;
        gp2_Frame = rhs.gp2_Frame;
        ext_Frame = rhs.ext_Frame;
    }
public:
    std::shared_ptr<SimFrame> gp1_Frame = nullptr;
    std::shared_ptr<SimFrame> gp2_Frame = nullptr;
    std::shared_ptr<SimFrame> ext_Frame = nullptr;
};

enum class PathType{
    ComposCart = 0,
    LinerSegment,
    SinglePoint,
    JointSpace
};

// TODO: Refactory, GroupFrame CordInfo is abandoned when creating/add into a GroupPath
class GroupPath{
public:
    PathType m_Type;
    KDL::Path_RoundedComposite* gp1_RoundPath = nullptr;
    KDL::Path_RoundedComposite* gp2_RoundPath = nullptr;

    KDL::Path_Line* gp1_LinerPath = nullptr;
    KDL::Path_Line* gp2_LinerPath = nullptr;

    KDL::Path_Point* gp1_PointPath = nullptr;
    KDL::Path_Point* gp2_PointPath = nullptr;

    KDL::VelocityProfile* gp1_vprf = nullptr;
    KDL::VelocityProfile* gp2_vprf = nullptr;

    std::shared_ptr<KDL::Path_JointSpace> ext_Path = nullptr;
    std::shared_ptr<KDL::VelocityProfile_JointSpace> extJnts_JntVprf = nullptr;

    // For Joint Space
    std::shared_ptr<KDL::Path_JointSpace> group_JointPath = nullptr;
    std::shared_ptr<KDL::VelocityProfile_JointSpace> group_JntVprf = nullptr;

    std::vector<double> path_Duration = std::vector<double>(4,-1.0);

public:
    GroupPath(const std::shared_ptr<GroupFrame> _1stFrame,
              const std::shared_ptr<GroupFrame> _2ndFrame,
              double speed, double acc,
              double bl, double v_bl){
        m_Type = PathType::ComposCart;
//        KDL::Path
        if(_1stFrame->gp1_Frame != nullptr && _2ndFrame->gp1_Frame != nullptr){
            gp1_vprf = new KDL::VelocityProfile_Trap(speed, acc);
            gp1_RoundPath = new KDL::Path_RoundedComposite(bl,bl,
                                                           new KDL::RotationalInterpolation_SingleAxis());
            gp1_RoundPath->Add(_1stFrame->gp1_Frame->frame_Cart);
            gp1_RoundPath->Add(_2ndFrame->gp1_Frame->frame_Cart);
        }

        if(_1stFrame->gp2_Frame != nullptr && _2ndFrame->gp2_Frame != nullptr){
            gp2_vprf = new KDL::VelocityProfile_Trap(speed, acc);
            gp2_RoundPath = new KDL::Path_RoundedComposite(bl,bl,
                                                           new KDL::RotationalInterpolation_SingleAxis());
            gp2_RoundPath->Add(_1stFrame->gp2_Frame->frame_Cart);
            gp2_RoundPath->Add(_2ndFrame->gp2_Frame->frame_Cart);
        }

        ext_Path = std::make_shared<KDL::Path_JointSpace>(_1stFrame->ext_Frame->frame_Jnts,
                                                          _2ndFrame->ext_Frame->frame_Jnts);
    }

    void addFrame(const std::shared_ptr<GroupFrame> n_Frame){
        if(m_Type != PathType::ComposCart)
            return;
        if(gp1_RoundPath != nullptr)
            gp1_RoundPath->Add(n_Frame->gp1_Frame->frame_Cart);
        if(gp2_RoundPath != nullptr)
            gp2_RoundPath->Add(n_Frame->gp2_Frame->frame_Cart);
        ext_Path->Add(n_Frame->ext_Frame->frame_Jnts);
    }

    void FinishPath(const std::vector<double>& extAxisSpeeds){
        if(m_Type != PathType::ComposCart)
            return;
        if(gp1_RoundPath != nullptr){
            gp1_RoundPath->Finish();
            gp1_vprf->SetProfile(0, gp1_RoundPath->PathLength());
        }

        if(gp2_RoundPath != nullptr){
            gp2_RoundPath->Finish();
            gp2_vprf->SetProfile(0, gp2_RoundPath->PathLength());
        }
        extJnts_JntVprf = std::make_shared<KDL::VelocityProfile_JointSpace>(extAxisSpeeds);
        updatePathDuration();
    }


    GroupPath(const std::shared_ptr<GroupFrame> _1stFrame,
              const std::shared_ptr<GroupFrame> _2ndFrame,
              double speed, double acc,
              const std::vector<double>& extAxisSpeeds){
        m_Type = PathType::LinerSegment;
        if(_1stFrame->gp1_Frame != nullptr && _2ndFrame->gp1_Frame != nullptr){
            gp1_vprf = new KDL::VelocityProfile_Trap(speed, acc);

            gp1_LinerPath = new KDL::Path_Line(_1stFrame->gp1_Frame->frame_Cart,
                                               _2ndFrame->gp1_Frame->frame_Cart,
                                               new KDL::RotationalInterpolation_SingleAxis(),
                                               1.0, true);
            gp1_vprf->SetProfile(0, gp1_LinerPath->PathLength());
        }


        if(_1stFrame->gp2_Frame != nullptr && _2ndFrame->gp2_Frame != nullptr){
            gp2_vprf = new KDL::VelocityProfile_Trap(speed, acc);
            gp2_LinerPath = new KDL::Path_Line(_1stFrame->gp2_Frame->frame_Cart,
                                               _2ndFrame->gp2_Frame->frame_Cart,
                                               new KDL::RotationalInterpolation_SingleAxis(),
                                               1.0, true);
            gp2_vprf->SetProfile(0, gp2_LinerPath->PathLength());
        }


        ext_Path = std::make_shared<KDL::Path_JointSpace>(_1stFrame->ext_Frame->frame_Jnts,
                                                          _2ndFrame->ext_Frame->frame_Jnts);
        extJnts_JntVprf = std::make_shared<KDL::VelocityProfile_JointSpace>(extAxisSpeeds);
        updatePathDuration();
    }

    // Single Point
    GroupPath(const std::shared_ptr<GroupFrame> singleFrame,
              double speed = 0, double acc = 0,
              const std::vector<double> extAxisSpeeds = std::vector<double>(8,0.0),
              double duration = 0){
        m_Type = PathType::SinglePoint;
        if(singleFrame->gp1_Frame!=nullptr){
            gp1_PointPath = new KDL::Path_Point(singleFrame->gp1_Frame->frame_Cart);
            gp1_vprf = new KDL::VelocityProfile_Trap(speed, acc);
            path_Duration[0] = duration;

        }
        if(singleFrame->gp2_Frame!=nullptr){
            gp2_PointPath = new KDL::Path_Point(singleFrame->gp2_Frame->frame_Cart);
            gp2_vprf = new KDL::VelocityProfile_Trap(speed, acc);
            path_Duration[1] = duration;
        }

        ext_Path = std::make_shared<KDL::Path_JointSpace>(singleFrame->ext_Frame->frame_Jnts);
        extJnts_JntVprf = std::make_shared<KDL::VelocityProfile_JointSpace>(extAxisSpeeds);
        path_Duration[2] = duration;
    }

    const std::vector<double>& getPathDefaultDuration() const{
        return path_Duration;
    }

    void updatePathDuration(){
        switch(m_Type){

        case PathType::ComposCart:
            if(gp1_RoundPath != nullptr){
                auto t_Trac = new KDL::Trajectory_Segment(gp1_RoundPath,gp1_vprf);
                path_Duration[0] = t_Trac->Duration();
            }
            if(gp2_RoundPath != nullptr){
                auto t_Trac = new KDL::Trajectory_Segment(gp2_RoundPath,gp2_vprf);
                path_Duration[1] = t_Trac->Duration();
            }
            break;
        case PathType::LinerSegment:
            if(gp1_LinerPath != nullptr){
                auto t_Trac = new KDL::Trajectory_Segment(gp1_LinerPath,gp1_vprf);
                path_Duration[0] = t_Trac->Duration();
            }
            if(gp2_LinerPath != nullptr){
                auto t_Trac = new KDL::Trajectory_Segment(gp2_LinerPath,gp2_vprf);
                path_Duration[1] = t_Trac->Duration();
            }
            break;
//        case PathType::SinglePoint:
//            if(gp1_PointPath != nullptr){
//                auto t_Trac = new KDL::Trajectory_Segment(gp1_PointPath,gp1_vprf);
//                path_Duration[0] = t_Trac->Duration();
//            }
//            if(gp2_PointPath != nullptr){
//                auto t_Trac = new KDL::Trajectory_Segment(gp2_PointPath,gp2_vprf);
//                path_Duration[1] = t_Trac->Duration();
//            }
//            break;
        case PathType::JointSpace:
            if(group_JointPath!=nullptr){
                auto t_Trac = new KDL::Trajectory_JointSpace(group_JointPath,group_JntVprf);
                path_Duration[3] = t_Trac->Duration();
            }
            break;
        }
        if(ext_Path != nullptr){
            auto t_Trac = new KDL::Trajectory_JointSpace(ext_Path,extJnts_JntVprf);
            path_Duration[2] = t_Trac->Duration();
        }
    }

    const bool group1Valid() const{
        return gp1_RoundPath!=nullptr || gp1_LinerPath!=nullptr || gp1_PointPath!=nullptr;
    }
    const bool group2Valid() const{
        return gp2_RoundPath!=nullptr || gp2_LinerPath!=nullptr || gp2_PointPath!=nullptr;
    }

    bool groupJointPathValid(){
        return group_JointPath!=nullptr;
    }

};


// TODO: Refactory, generate Trac on-fly, based on path Vector
class GroupTrac{

    PathType m_Type;
    KDL::Trajectory_Composite* gp1_CartTrac = nullptr;
    KDL::Trajectory_Composite* gp2_CartTrac = nullptr;
    JntTrac_sptr ext_JntsTrac = nullptr;
    JntTrac_sptr grp_JntsTrac = nullptr;
    std::vector<std::shared_ptr<GroupPath>> path_Vec;
    double m_Duration;

public:
    GroupTrac(){
        gp1_CartTrac = new KDL::Trajectory_Composite();
        gp2_CartTrac = new KDL::Trajectory_Composite();
        ext_JntsTrac = std::make_shared<KDL::Trajectory_JointSpace>();
        grp_JntsTrac = std::make_shared<KDL::Trajectory_JointSpace>();
    }
    ~GroupTrac(){}
    void Add(const std::shared_ptr<GroupPath> t_PathPtr, bool flag_syncronize = true){
        auto t_TracDuration = t_PathPtr->getPathDefaultDuration();
        if(flag_syncronize){
            double t_MinDuration = -1.0;
            for(auto dT : t_PathPtr->getPathDefaultDuration())
                t_MinDuration = std::max(t_MinDuration, dT);
            t_TracDuration = std::vector<double>(4, t_MinDuration);
        }

        switch(t_PathPtr->m_Type){
        case PathType::ComposCart:{
            if(t_PathPtr->group1Valid()){
                gp1_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp1_RoundPath,
                                                              t_PathPtr->gp1_vprf,
                                                              t_TracDuration[0]));
            }

            if(t_PathPtr->group2Valid()){
                gp2_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp2_RoundPath,
                                                              t_PathPtr->gp2_vprf,
                                                              t_TracDuration[1]));
            }
            if(t_PathPtr->ext_Path->isValid()){
                ext_JntsTrac->Add(t_PathPtr->ext_Path,
                                  t_PathPtr->extJnts_JntVprf,
                                  t_TracDuration[2]);
            }
        }
            break;
        case PathType::LinerSegment:{
            if(t_PathPtr->group1Valid()){
                gp1_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp1_LinerPath,
                                                              t_PathPtr->gp1_vprf,
                                                              t_TracDuration[0]));
            }
            if(t_PathPtr->group2Valid()){
                gp2_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp2_LinerPath,
                                                              t_PathPtr->gp2_vprf,
                                                              t_TracDuration[1]));
            }
            if(t_PathPtr->ext_Path->isValid()){
                ext_JntsTrac->Add(t_PathPtr->ext_Path,
                                  t_PathPtr->extJnts_JntVprf,
                                  t_TracDuration[2]);
            }
        }
            break;
        case PathType::SinglePoint:{
            if(t_PathPtr->group1Valid()){
                gp1_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp1_PointPath,
                                                              t_PathPtr->gp1_vprf,
                                                              t_TracDuration[0]));
            }
            if(t_PathPtr->group2Valid()){
                gp2_CartTrac->Add(new KDL::Trajectory_Segment(t_PathPtr->gp2_PointPath,
                                                              t_PathPtr->gp2_vprf,
                                                              t_TracDuration[1]));
            }
            if(t_PathPtr->ext_Path->isValid()){
                ext_JntsTrac->Add(t_PathPtr->ext_Path,
                                  t_PathPtr->extJnts_JntVprf,
                                  t_TracDuration[2]);
            }
        }
            break;

        case PathType::JointSpace:
            if(t_PathPtr->group_JointPath->isValid())
                grp_JntsTrac->Add(t_PathPtr->group_JointPath,
                                  t_PathPtr->group_JntVprf,
                                  t_TracDuration[3]);
            break;
        }
        updateTracDuration(t_PathPtr->m_Type);


    }
    const double getTracDuration() const;
    const GroupPose getGroupPose(const double t) const{
        GroupPose t_Pose;
//        switch(m_Type){
//        case PathType::ComposCart:
//        case PathType::LinerSegment:
//            if(gp1_CartTrac!=nullptr)
//                t_Pose.Pose_Rbt1.setPoseData(toPlacement(gp1_CartTrac->Pos(t)));
//            if(gp2_CartTrac!=nullptr)
//                t_Pose.Pose_Rbt2.setPoseData(toPlacement(gp2_CartTrac->Pos(t)));
//            t_Pose.setExtVals(ext_JntsTrac->Pos(t));
//            break;
//        case PathType::JointSpace:{
//            auto t_JntVals = Robot::toVector(grp_JntsTrac->Pos(t));
//        }
//            break;
//        }
        if(m_Type != PathType::JointSpace){
            if(gp1_CartTrac!=nullptr)
                t_Pose.Pose_Rbt1.setPoseData(toPlacement(gp1_CartTrac->Pos(t)));
            if(gp2_CartTrac!=nullptr)
                t_Pose.Pose_Rbt2.setPoseData(toPlacement(gp2_CartTrac->Pos(t)));
            if(ext_JntsTrac!=nullptr)
                t_Pose.setExtVals(ext_JntsTrac->Pos(t));
        }
        return t_Pose;
    }

protected:
    void updateTracDuration(const PathType& t_Type){
        if(t_Type == PathType::JointSpace){
            m_Duration = grp_JntsTrac->Duration();
            return;
        }
        if(gp1_CartTrac)
            m_Duration = std::max(gp1_CartTrac->Duration(),m_Duration);
        if(gp2_CartTrac)
            m_Duration = std::max(gp2_CartTrac->Duration(),m_Duration);
        m_Duration = std::max(ext_JntsTrac->Duration(),m_Duration);
    }
    bool syncronizeTrac(){
        return false;
    }
};



class RobotExport TracSimulator
{
    App::Document* m_DocPtr;
    // Operator
    Robot::MechanicGroup* m_GroupPtr = nullptr;
    GroupPose m_InitPose;
    GroupPose m_PoseBuffer;

    // Program Source Data
    RobotProg_sptr m_TargetProgramPtr = nullptr;
    std::vector<RobotCommand_sptr> m_MovCmdBuffer;

    // Trac Data(for simulation)
    std::shared_ptr<GroupTrac> m_SimTracBuffer = nullptr;
//    std::vector<std::shared_ptr<GroupTrac>> m_TracVec;

    double pastTime = 0.0;
    // To be Optimize
    std::list<std::shared_ptr<RobotProgram>> m_SimTargetList;
    bool flag_simFromCurrentPose = false;

public:
    /// Constructor
    TracSimulator();
    virtual ~TracSimulator();

    bool initDocPtr(App::Document* t_DocPtr);
    void setTargetSimProgram(const RobotProg_sptr t_Program);
    const RobotProg_sptr& getCurrentSimProgram() const;

    // Trac Simulation, Time Based
    int  generateSimulationTrac_FromSelectedCommand(const int s_ID = 0);
    const double getDuration (int n=-1) const;
    const RobotCommand_sptr getSelectedCommandPtr(const size_t t_ID) const ;
    const RobotWaypoint_sptr getSelectedCommandPose(const RobotCommand_sptr t_CmdPtr) const;
    const size_t getTotalCommandNumber() const;
    const size_t getMoveBufferCommandNum() const;
    bool executeSelectedCommand(const RobotCommand_sptr t_CmdPtr);
    bool executeToTime(const double t_Time);
    bool updatePoseBuffer_ByTime(double time);
    bool isCommandIDValid(const size_t t_ID);
    const size_t getMatchedCommandID(const double t_Time, const size_t t_CmdID);
    bool udpateGroupPoseByTime(const double t_Time);
    bool commandFinished(const size_t t_CmdID);
    void resetSimulator(void);

    // Trac Simulation, Command Based
    int  generateSimulationTrac_FromCurrentPose();
    void updateMovCommandBuffer(const std::vector<RobotCommand_sptr>& t_cmdBuffer);

protected:
    bool moveCommandFinished(MoveCommand *t_cmmdPtr);
};



} //namespace Robot



#endif
