#pragma once

#include <boost/filesystem.hpp>
#include <Mod/Robot/App/Trac/RobotProgram.h>

using RobotProg_sptr = std::shared_ptr<Robot::RobotProgram>;

class ExportUtility{
    static bool exportRobotProgram_Googol_V0428(const string file_Path,
                                                const RobotProg_sptr t_Prog);

};
