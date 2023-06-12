// Created By Yixiao 2022-06-21

#ifndef ROBOT_MOVECOMMAND_H
#define ROBOT_MOVECOMMAND_H

#include "CommandBase.h"


namespace Robot
{

enum class SpeedType{
    Percent = 0,
    TrueVal
};

enum class MoveType{
    STOP = 0,
    MOVJ,
    MOVL,
    MOVC
};

enum class MovePrec{
    FINE,
    CNT
};

class RobotExport MoveCommand : public CommandBase
{
    TYPESYSTEM_HEADER();

public:
    MoveCommand();
    MoveCommand(const MoveCommand&);
    ~MoveCommand();

    MoveCommand(const MoveType& t_Type,
                const MovePrec& t_Prec,
                const size_t pntID,
                const float t_V,
                const float t_BL,
                const float t_VBL);

    MoveCommand &operator=(const MoveCommand&);

    const std::size_t& getMovPoseID() const{
        return m_PoseID;
    }

    const MoveType& getMoveType() const{
        return m_MovType;
    }
    const MovePrec& getMovePrec() const{
        return m_MovPrec;
    }
    const std::vector<float> getMoveParam() const{
        std::vector<float> result;
        result.push_back(m_Vel);
        result.push_back(m_BL);
        result.push_back(m_VBL);
        return result;
    }

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

protected:
    MoveType m_MovType = MoveType::STOP;
    MovePrec m_MovPrec = MovePrec::FINE;
    SpeedType m_SpeedType = SpeedType::TrueVal;
    size_t m_PoseID = -1;    // MOVC need 2 points?
    float m_Vel = 0.0;
    float m_BL = 0.0;
    float m_VBL = 0.0;
};

} //namespace Part


#endif // PART_TOPOSHAPE_H
