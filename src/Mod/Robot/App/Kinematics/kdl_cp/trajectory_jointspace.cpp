// Created By Yixiao 2022-08-23

#include "trajectory_jointspace.h"

using namespace KDL;


Trajectory_JointSpace::Trajectory_JointSpace()
{
    m_JPath = std::make_shared<KDL::Path_JointSpace>();
}

Trajectory_JointSpace::Trajectory_JointSpace(std::shared_ptr<Path_JointSpace> t_PathPtr, std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr_limit)
{
    m_JPath = std::make_shared<KDL::Path_JointSpace>();
    Add(t_PathPtr, t_vlpfPtr_limit);
}

bool Trajectory_JointSpace::Add(std::shared_ptr<Path_JointSpace> t_PathPtr,
                                std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
                                double t_Duration)
{
    if(m_Dof == 0)
        m_Dof = t_PathPtr->getPathDOF();
    else if(m_Dof != t_PathPtr->getPathDOF())
        return false;

    double p_Duration = calcTracMinExecutionTime(t_PathPtr,t_vlpfPtr);
    if(p_Duration < t_Duration){
        p_Duration = adjustTracDuration(t_PathPtr,t_vlpfPtr,p_Duration,t_Duration)?t_Duration:p_Duration;
    }
    appendTargetPath(t_PathPtr,t_vlpfPtr, p_Duration);
    return true;
}

const JntArray Trajectory_JointSpace::Pos(double t_sec) const
{
    JntArray jntPose(m_Dof);
    if(t_sec > m_TracDuration || t_sec < 0 || m_Tvec.empty())
        return jntPose;
    int f_ID = 0, s_ID = 0;
    double passed_T = 0, dT = 0.0;
    for(auto t_Ft : m_Tvec){
        passed_T += t_Ft;
        if(t_sec < passed_T){
            dT = std::abs(passed_T - t_Ft - t_sec);
            break;
        }
        else{
            f_ID++;
        }
    }
    if(f_ID == m_Tvec.size()){
        s_ID = f_ID;
    }else{
        s_ID = f_ID+1;
    }
    auto base_Frame = m_JPath->getJFrames()[f_ID];
    auto next_Frame = m_JPath->getJFrames()[s_ID];
//    auto dT = total_dT - t_sec;
    JntArray incAngles(m_Dof);
    JntArray distance(m_Dof);

    KDL::Subtract(next_Frame,base_Frame,distance);
    JntArray t_JntVels(m_JVelVec[f_ID]);
    for(int i = 0; i<distance.rows();i++){
        if(distance.data(i)<0)
            t_JntVels.data(i)*=-1;
    }
    KDL::Multiply(t_JntVels,dT,incAngles);
    KDL::Add(base_Frame, incAngles, jntPose);
    return jntPose;
}

const double Trajectory_JointSpace::Duration() const
{
    return m_TracDuration;
}

// MinDt = Sum(Max(dFt))
double Trajectory_JointSpace::calcTracMinExecutionTime(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                                                       const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr)
{
    if(t_PathPtr == nullptr || t_vlpfPtr == nullptr)
        return false;
    double totalDuration = 0;
    m_Tvec_Buffer.clear();
    m_JVelVec_Buffer.clear();

    auto newFrames = t_PathPtr->getJFrames();
    if(newFrames.size()<2)
        return 0;
    std::vector<JntArray> t_Frames;
    if(!m_JPath->getJFrames().empty()){
        t_Frames.push_back(m_JPath->getJFrames().back());   // Insert the Last frame of old Jpath
        t_Frames.resize(1+newFrames.size());
        std::copy(newFrames.begin(),newFrames.end(),t_Frames.begin()+1);
    }else{
        t_Frames = newFrames;
    }

    int i = 0;
    JntArray t_Vel = t_vlpfPtr->getJntVels();;
    auto _1stFrameIter = t_Frames.begin();
    auto _2ndFrameIter = t_Frames.begin()+1;
    while(_2ndFrameIter!= t_Frames.end()){
        auto dT = calculateSegmentDuration(*_1stFrameIter,
                                           *_2ndFrameIter,
                                           t_Vel);
        auto t_Vel = calcRequiredJntSpeed(*_1stFrameIter,
                                          *_2ndFrameIter,
                                          dT);
        m_Tvec_Buffer.push_back(dT);
        m_JVelVec_Buffer.push_back(t_Vel);
        totalDuration += dT;
        _1stFrameIter = _2ndFrameIter;
        _2ndFrameIter++;
        i++;
    }

    return totalDuration;
}

const double Trajectory_JointSpace::calculateSegmentDuration(const JntArray &j1,
                                                             const JntArray &j2,
                                                             const JntArray &j_Vels)
{
    double max_dT = -1.0;
    if(j1.rows() != j2.rows() ||
       j1.rows() != j_Vels.rows())
        return max_dT;
    JntArray distance, t_jVels;
    distance.resize(j1.rows());
    t_jVels.resize(j1.rows());
    KDL::Subtract(j2,j1,distance);
    for(int i = 0 ; i<distance.rows();i++){
        if(j_Vels(i) == 0)
            continue;
        auto dT = std::abs(distance.data(i)/j_Vels.data(i));
        if(dT > max_dT)
            max_dT = dT;
    }
    return max_dT;
}

bool Trajectory_JointSpace::adjustTracDuration(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                                               const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
                                               const double min_Duration,
                                               const double tar_Duration)
{
    if(t_PathPtr == nullptr || t_vlpfPtr == nullptr)
        return false;

    auto newFrames = t_PathPtr->getJFrames();
    if(newFrames.size()<2)
        return false;
    std::vector<JntArray> t_Frames;
    if(!m_JPath->getJFrames().empty()){
        t_Frames.push_back(m_JPath->getJFrames().back());   // Insert the Last frame of old Jpath
        t_Frames.resize(1+newFrames.size());
        std::copy(newFrames.begin(),newFrames.end(),t_Frames.begin()+1);
    }else{
        t_Frames = newFrames;
    }


//    m_JVelVec_Buffer.clear();
    bool fillBlank = false;
    if(min_Duration == 0)
        fillBlank = true;
    auto ratio = tar_Duration/min_Duration;
    int i = 0;


    auto _1stFrameIter = t_Frames.begin();
    auto _2ndFrameIter = t_Frames.begin()+1;
    double t_dT = 0.0;
    while(_2ndFrameIter!= t_Frames.end()){
        if(!fillBlank)
            t_dT = ratio*m_Tvec_Buffer[i];
        else
            t_dT = tar_Duration/(t_Frames.size()-1);
        JntArray t_Vel = calcRequiredJntSpeed(*_1stFrameIter,
                                              *_2ndFrameIter,
                                              t_dT);
        m_JVelVec_Buffer[i] = t_Vel;
        m_Tvec_Buffer[i] = t_dT;
        _1stFrameIter = _2ndFrameIter;
        _2ndFrameIter++;
        i++;
    }

    return true;
}

void Trajectory_JointSpace::appendTargetPath(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                                             const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
                                             const double path_duration)
{
    m_JPath->Add(t_PathPtr);

    auto o_tsize = m_Tvec.size();
    m_Tvec.resize(o_tsize + m_Tvec_Buffer.size());
    std::copy(m_Tvec_Buffer.begin(), m_Tvec_Buffer.end(),m_Tvec.begin()+o_tsize);

    auto o_vsize = m_JVelVec.size();
    m_JVelVec.resize(o_vsize + m_JVelVec_Buffer.size());
    std::copy(m_JVelVec_Buffer.begin(), m_JVelVec_Buffer.end(),m_JVelVec.begin()+o_vsize);

    m_JVelLimit = t_vlpfPtr;
    m_TracDuration += path_duration;
}

const JntArray Trajectory_JointSpace::calcRequiredJntSpeed(const JntArray &src,
                                                           const JntArray &dst,
                                                           const double dT)
{
    JntArray distance;
    distance.resize(src.rows());
    KDL::Subtract(dst,src,distance);
    JntArray t_jVels;
    t_jVels.resize(distance.rows());
    KDL::SetToZero(t_jVels);
    if(dT<=0)
        return t_jVels;
    KDL::Divide(distance, dT, t_jVels);
     for(int i = 0; i<t_jVels.rows(); i++){
         t_jVels.data(i) = std::abs(t_jVels.data(i));
     }
     return t_jVels;
}



