// Created By Yixiao 2022-06-22

#ifndef ROBOT_TOOLCOMMAND_H
#define ROBOT_TOOLCOMMAND_H

#include "Mod/Robot/App/Tool/ToolObject.h"
//#include "CommandBase.h"
#include "CoordCommand.h"
#include <memory>

namespace Robot
{

class RobotExport ToolCommand : public CoordCommand
{
    TYPESYSTEM_HEADER();

public:
    ToolCommand();
    ToolCommand(const ToolCommand&);
    // Switch Tool
    ToolCommand(const ToolType& t_Type,
                const uint coordID);
    // Turn On/Off Tool
    ToolCommand(const ToolType& t_Type,
                const bool status);

    ~ToolCommand();

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);
    const ToolType& getToolType() const{
        return m_ToolType;
    }
    const bool getToolStatus() const{
        return m_status;
    }
protected:
    ToolType m_ToolType;
    bool m_status;
};

}


#endif
