#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "Export_Utility.h"

//using boost::filesystem;

bool ExportUtility::exportRobotProgram_Googol_V0428(const std::string file_Path,
                                                    const RobotProg_sptr t_Prog)
{
    bool success;
    success = t_Prog!=nullptr;
    success &= t_Prog->isProgramValid();
    auto t_Path = boost::filesystem::path(file_Path);
    if(boost::filesystem::is_empty(t_Path))
       success &= boost::filesystem::create_directory(t_Path);
    if(!success)
        return false;

    std::vector<RobotCommand_sptr> freeTrac_Buffer;
    std::vector<RobotCommand_sptr> weldTrac_Buffer;
    std::vector<RobotCommand_sptr> scanTrac_Buffer;

    for(const auto& t_cmdPtr : t_Prog->getCmmdData()){



    }
}

