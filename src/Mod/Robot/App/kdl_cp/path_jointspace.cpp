// Created By Yixiao 2022-08-23

#include "path_jointspace.h"

using namespace KDL;

Path_JointSpace::Path_JointSpace()
{
}

Path_JointSpace::~Path_JointSpace()
{

}

Path_JointSpace::Path_JointSpace(const JntArray &f1, const JntArray &f2)
{
    if(f1.rows() != f2.rows())
        return;
    m_NbrDOF = f1.rows();
    m_JFrames.push_back(f1);
    m_JFrames.push_back(f2);
}

Path_JointSpace::Path_JointSpace(const JntArray &singleFrame)
{
    m_NbrDOF = singleFrame.rows();
    m_JFrames.push_back(singleFrame);
}

bool Path_JointSpace::isValid() const
{
    return m_NbrDOF != 0;
}

bool Path_JointSpace::Add(const JntArray &n_Frame)
{
    if(n_Frame.rows() != m_NbrDOF)
        return false;
    m_JFrames.push_back(n_Frame);
}

bool Path_JointSpace::Add(const std::shared_ptr<Path_JointSpace> t_PathPtr)
{
    if(m_NbrDOF == 0)
        m_NbrDOF = t_PathPtr->getPathDOF();
    else{
        if(t_PathPtr->getPathDOF() != m_NbrDOF)
            return false;
    }
    auto t_Frames = t_PathPtr->getJFrames();
    auto o_Size = m_JFrames.size();
    m_JFrames.resize(o_Size + t_Frames.size());
    std::copy(t_Frames.begin(),t_Frames.end(),m_JFrames.begin()+o_Size);
    return true;
}

const std::vector<JntArray> &Path_JointSpace::getJFrames() const
{
    return m_JFrames;
}

const uint Path_JointSpace::getPathDOF() const
{
    return m_NbrDOF;
}
