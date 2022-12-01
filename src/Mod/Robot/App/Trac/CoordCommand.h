// Created By Yixiao 2022-06-22

#ifndef ROBOT_CORDCOMMAND_H
#define ROBOT_CORDCOMMAND_H

#include "Mod/Robot/App/Utilites/FrameObject.h"
#include "Mod/Robot/App/Tool/ToolObject.h"
#include "RobotCommand.h"
#include <memory>

namespace Robot
{

class RobotExport CoordCommand : public RobotCommand
{
    TYPESYSTEM_HEADER();

public:
    CoordCommand();
    CoordCommand(const CoordCommand&);
    ~CoordCommand();

    CoordCommand(const std::string &taskName,
                 const std::string &executorName,
                 const CordType& t_Coord,
                 const uint t_ID);

    CoordCommand &operator=(const CoordCommand&);

    const CordType& getCoordType() const{
        return m_CoordType;
    }
    const uint getCoordID() const{
        return m_CoordID;
    }

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);
protected:
    CordType m_CoordType;
    uint m_CoordID;
};

}


#endif
