#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "FileIO_Utility.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <boost/filesystem.hpp>

FileIO_Utility::FileIO_Utility()
{
}

/// <summary>
/// 按顺序拆分程序
/// </summary>
/// <param name="t_Cmds"></param>
/// <returns></returns>
bool FileIO_Utility::splitProgramBySequence()
{
    auto t_Cmds = m_ProgramPtr->getCmmdData();
	if (t_Cmds.empty())
		return false;

	m_Vec_JOIN.clear();
	m_Vec_SCAN.clear();
	m_Vec_WELD.clear();

	// 找到所有 扫描/焊接 指令的命令行位置
	std::vector<int> vec_scanCmdLineID;
	std::vector<int> vec_weldCmdLineID;
	int cmdID_count = 0;
    for (auto cmdPtr : t_Cmds) {
		cmdID_count++;
        if (cmdPtr->getType() == Robot::CommandType::OptTool) {
            auto toolCmdPtr = static_cast<Robot::ToolCommand*>(cmdPtr.get());
            switch(toolCmdPtr->getToolType()){
            case Robot::ToolType::_2DScanner:
                // Laser On
                if(toolCmdPtr->getToolStatus()){
                    vec_scanCmdLineID.push_back(cmdID_count - 1);
                }
                break;
            case Robot::ToolType::WeldTorch:
                // Arc On
                if(toolCmdPtr->getToolStatus()){
                    vec_weldCmdLineID.push_back(cmdID_count);
                }
                // Arc Off
                else{
                    vec_weldCmdLineID.push_back(cmdID_count);
                }
                break;
            default:
                break;
            };
		}
	}

	int segmentID = 0;
	cmdID_count = 0;
	bool segment_scan = false;
	bool segment_weld = false;
	bool segment_arcoff = false;

    std::vector<RobotCommand_sptr> buffer_SCAN;
    std::vector<RobotCommand_sptr> buffer_WELD;
    std::vector<RobotCommand_sptr> buffer_ARCOFF;

	for (const auto& cmd_line : t_Cmds) {
		cmdID_count++;
		//if (cmd_line.t_Type == Cmd_type::CHANGE_COORDSYS)
		//	continue;
		auto res_1 = std::find(vec_scanCmdLineID.begin(), vec_scanCmdLineID.end(), cmdID_count);
        auto res_2 = std::find(vec_weldCmdLineID.begin(), vec_weldCmdLineID.end(), cmdID_count);

		if (res_1 != vec_scanCmdLineID.end()) {
			segment_scan = true;
			segment_weld = false;
			segment_arcoff = false;
			if (!buffer_ARCOFF.empty()) {
				m_Vec_JOIN.push_back(std::make_pair(segmentID, buffer_ARCOFF));
				buffer_ARCOFF.clear();
			}
			if (!buffer_SCAN.empty()) {
				m_Vec_SCAN.push_back(std::make_pair(segmentID, buffer_SCAN));
				buffer_SCAN.clear();
			}
			segmentID++;
		}

		if (res_2 != vec_weldCmdLineID.end()) {
			if (!segment_weld) {
				segment_weld = true;
				segment_arcoff = false;
				segment_scan = false;
			}
			else {
				segment_weld = false;
				segment_arcoff = true;
				if (!buffer_WELD.empty()) {
					m_Vec_WELD.push_back(std::make_pair(segmentID, buffer_WELD));
					buffer_WELD.clear();
				}
			}
		}

		if (segment_scan) {
			buffer_SCAN.push_back(cmd_line);
		}
		else if (segment_weld) {
			buffer_WELD.push_back(cmd_line);
		}
		else {
			buffer_ARCOFF.push_back(cmd_line);
		}
	}

	if (!buffer_ARCOFF.empty()) {
		m_Vec_JOIN.push_back(std::make_pair(segmentID, buffer_ARCOFF));
		buffer_ARCOFF.clear();
	}
	if (!buffer_SCAN.empty()) {
		m_Vec_SCAN.push_back(std::make_pair(segmentID, buffer_SCAN));
		buffer_SCAN.clear();
	}
	if (!buffer_WELD.empty()) {
		m_Vec_WELD.push_back(std::make_pair(segmentID, buffer_WELD));
		buffer_WELD.clear();
	}

    //分割文件
    bool nopartmissing = !m_Vec_JOIN.empty() && !m_Vec_SCAN.empty() && !m_Vec_WELD.empty();//无零件缺失
    bool AllPartsInSequence = (m_Vec_WELD.size() == m_Vec_SCAN.size()) && (m_Vec_SCAN.size() == m_Vec_JOIN.size() - 1);//所有部位序列编号匹配
    return nopartmissing && AllPartsInSequence;
}

std::shared_ptr<Robot::RobotProgram> FileIO_Utility::readinProgramFromDrive(const string &t_filePath,
                                                                            const string &t_executror)
{
    auto t_Path = boost::filesystem::path(t_filePath);
    if (!boost::filesystem::exists(t_Path))
        return nullptr;
    std::ifstream m_freader(t_Path.c_str(), std::ios::in);
    if(!m_freader.is_open())
        return nullptr;
    auto t_ProgramPtr = std::make_shared<Robot::RobotProgram>();
    std::string str_buffer;
    while(getline(m_freader, str_buffer)){
        // NOT MOVE COMMAND
        if(str_buffer.size()<100){
            auto setup_cmd = stdStringSplit(str_buffer,std::string(" "),false);
            if(setup_cmd.front() == std::string("SPEED")){
                //                m_ProgramPtr->insertCMD_SetSpeed();
            }
            else if(setup_cmd.front() == std::string("DYN")){

            }
            else if(setup_cmd.front() == std::string("COORD_NUM")){
                Robot::CordType t_Type;
                uint cordID = 0;
                auto cordType = stdStringSplit(setup_cmd[1],std::string("="),false);
                if(cordType[1] == std::string("TCS"))
                    t_Type = Robot::CordType::TCS;
                else if(cordType[1] == std::string("PCS"))
                    t_Type = Robot::CordType::PCS;
                else if(cordType[1] == std::string("MCS"))
                    t_Type = Robot::CordType::MCS;
                else if(cordType[1] == std::string("ACS"))
                    t_Type = Robot::CordType::ACS;
                else if(cordType[1] == std::string("VCS"))
                    t_Type = Robot::CordType::VCS;
                if(setup_cmd.size() == 3){
                    auto cordNum = stdStringSplit(setup_cmd[2],std::string("="),false);
                    cordID = std::atoi(cordNum[1].c_str());
                }
                t_ProgramPtr->insertCMD_ChgCord(t_Type, cordID);
            }
        }
        else{
            auto mov_cmd = stdStringSplit(str_buffer,std::string(" "),false);
            if(mov_cmd.size()!=5)
                continue;
            Robot::MoveType t_MovType;
            Robot::MovePrec t_MovPrec;
            float t_Vel, t_bl, t_vbl;
            if(mov_cmd[0] == std::string("MOVL")){
                t_MovType = Robot::MoveType::MOVL;
            }
            else if(mov_cmd[0] == std::string("MOVJ")){
                t_MovType = Robot::MoveType::MOVJ;
            }
            else if(mov_cmd[0] == std::string("MOVC")){
                t_MovType = Robot::MoveType::MOVC;
            }

            auto vel_cmd = stdStringSplit(mov_cmd[1],std::string("="),false);
            if(vel_cmd[1].back() == '%')
                vel_cmd[1].pop_back();
            t_Vel = std::atof(vel_cmd[1].c_str());

            auto bl_cmd = stdStringSplit(mov_cmd[2],std::string("="),false);
            t_bl = std::atof(bl_cmd[1].c_str());

            t_MovPrec = t_bl==0?Robot::MovePrec::FINE:Robot::MovePrec::CNT;

            auto vbl_cmd = stdStringSplit(mov_cmd[3],std::string("="),false);
            t_vbl = std::atof(vbl_cmd[1].c_str());

            auto pose_cmd = stdStringSplit(mov_cmd[4],std::string("$"),false);

            auto poseExtractor = [](const std::vector<std::string>& t_StrVec){
                Robot::RobotPose t_Pose;
                if(t_StrVec[0] == std::string("@0"))
                    t_Pose.CordInfo.first = Robot::CordType::PCS;
                else if(t_StrVec[0] == std::string("@1"))
                    t_Pose.CordInfo.first = Robot::CordType::MCS;
                else if(t_StrVec[0] == std::string("@2"))
                    t_Pose.CordInfo.first = Robot::CordType::ACS;
                else if(t_StrVec[0] == std::string("@3"))
                    t_Pose.CordInfo.first = Robot::CordType::TCS;
                t_Pose.ConfigID = std::atoi(t_StrVec[1].c_str());

                for(auto i = 0; i<8; i++){
                    t_Pose.PoseData[i] = std::atof(t_StrVec[i+8].c_str());
                }
                return t_Pose;
            };

            auto gp1_PoseStr = stdStringSplit(pose_cmd[0], std::string(","),false);
            auto gp2_PoseStr = stdStringSplit(pose_cmd[1], std::string(","),false);
            auto ext_PoseStr = stdStringSplit(pose_cmd[2], std::string(","),false);

            Robot::MechPose t_Pose;
//            t_Pose.Pose_Rbt1 = poseExtractor(gp1_PoseStr);
//            t_Pose.Pose_Rbt2 = poseExtractor(gp2_PoseStr);
//            for(auto i = 0; i<8; i++){
//                t_Pose.ExtVals[i] = std::atof(ext_PoseStr[i].c_str());
//            };

//            auto t_PoseID = t_ProgramPtr->addNewPose(Robot::RobotWaypoint(t_Pose));

//            t_ProgramPtr->insertCMD_NewMOVE(t_executror,
//                                            t_PoseID,
//                                            t_MovType,
//                                            t_MovPrec,
//                                            t_Vel,t_bl,t_vbl);
        }
    }

    m_freader.close();

    return t_ProgramPtr;
}


/// <summary>
/// 根据是否拆分，分别生成程序序列
/// </summary>
/// <param name="t_filePath"></param>
/// <returns></returns>
bool FileIO_Utility::generateProgramSequence(const std::string& t_filePath,
                                             std::shared_ptr<Robot::RobotProgram> t_bufferPtr)
{
    if(t_bufferPtr == nullptr)
        return false;
    m_ProgramPtr = t_bufferPtr;
	// 确保输出文件目录存在
    auto t_Path = boost::filesystem::path(t_filePath);
    if (!boost::filesystem::exists(t_Path)) {
        if (!boost::filesystem::create_directory(t_Path)) {
			return false;
		}
	}

	if (m_flag_Split) {
        if(!splitProgramBySequence())
            return false;
		// Generate xihu_x.prg
		for (const auto& segment : m_Vec_JOIN) {
			if (segment.first == 0)
				continue;
            progTranslator_new(ExportFileType::xihu_File,
                               t_filePath, segment.first, segment.second);
		}

		// Generate WD-xpro.prg
		for (const auto& segment : m_Vec_SCAN) {
            progTranslator_new(ExportFileType::scan_File,
                               t_filePath, segment.first, segment.second);
		}

		// Generate SeamTeach_x.prg
		for (const auto& segment : m_Vec_WELD) {
            progTranslator_new(ExportFileType::weld_File,
                               t_filePath, segment.first, segment.second);
		}
	}
	else {

        progTranslator_new(ExportFileType::test_File, t_filePath, 1, m_ProgramPtr->getCmmdData());
		//progTranslator(ExportFileType::test_File, t_filePath, 1, m_BufferPtr->getCmmdLineVec());
	}

	return true;
}




int FileIO_Utility::toolIDtranslate(const int t_ToolID)
{
	int result = 0;
    switch (t_ToolID)
	{
	case 1:
		result = m_scanToolID;
		break;
	case 2:			// 试驾场 焊枪ID号
		result = m_weldToolID;
	default:
		break;
	}
	return result;
}

/// <summary>
/// 程序转换器（新格式）
/// </summary>
/// <param name="t_FileType"></param>
/// <param name="basePath">输出路径</param>
/// <param name="sequenceID"></param>
/// <param name="t_cmds"></param>
/// <returns></returns>
bool FileIO_Utility::progTranslator_new(const ExportFileType& t_FileType,
                                        const std::string basePath,
                                        const int sequenceID,
                                        const std::vector<RobotCommand_sptr>& t_cmmdVec)
{
	// 确保输出文件目录存在
    auto t_Path = boost::filesystem::path(basePath);
    if (!boost::filesystem::exists(t_Path)) {
        if (!boost::filesystem::create_directory(t_Path)) {
            return false;
        }
    }

	//创建时间
	auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::stringstream ss;
	ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
	std::string Create_time = ss.str();

	std::string fileName, filePath;

    // Export Program Head
	switch (t_FileType)
	{
	case ExportFileType::scan_File: {
        fileName = std::string("\WD-") + std::to_string(sequenceID) + std::string("pro.prg");
		filePath = basePath + fileName;
		m_fwriter.open(filePath, std::ios::out);
		//固定格式
		m_fwriter << "//Project Name:   " + fileName << "\n";
		m_fwriter << "//Create Date:   " + Create_time << "\n";
		m_fwriter << "////////////////////////////////////////////////" << "\n";
		m_fwriter << "NOP" << "\n";
		m_fwriter << "SPEED SP=100%" << "\n";
		m_fwriter << "DYN ACC=100 DCC=100.0 J=250" << "\n";

        // First WD-1pro.prg will contain the program from safepoint to 1 seam start point
		auto segment_join = m_Vec_JOIN.front();
        exportCommand(segment_join.second);
        break;
    }
	case ExportFileType::weld_File: {
        fileName = std::string("\SeamTeach") + std::to_string(sequenceID) + std::string(".prg");
		filePath = basePath + fileName;
		m_fwriter.open(filePath, std::ios::out);
		//固定格式
		m_fwriter << "NOP" << "\n";
		m_fwriter << "SPEED SP=100%" << "\n";
		m_fwriter << "DYN ACC=80 DEC=80 J=250" << "\n";	// 加速/减速为100%，加速时间为192ms
        break;
    }
	case ExportFileType::xihu_File: {
        fileName = std::string("\Xihu") + std::to_string(sequenceID) + std::string(".prg");
		filePath = basePath + fileName;
		m_fwriter.open(filePath, std::ios::out);
		//固定格式
		m_fwriter << "//Project Name:   " + fileName << "\n";
		m_fwriter << "//Create Date:   " + Create_time << "\n";
		m_fwriter << "////////////////////////////////////////////////" << "\n";
		m_fwriter << "NOP" << "\n";
		m_fwriter << "SPEED SP=100.0" << "\n";
		m_fwriter << "DYN ACC=80 DEC=80 J=250" << "\n";	// 加速/减速为100%，加速时间为192ms
		m_fwriter << "COORD_NUM COOR=" << "TCS" << " ID=" << std::to_string(3) << "\n";
        break;
    }
	case ExportFileType::test_File: {
        fileName = std::string("\TestFile_") + std::to_string(sequenceID) + std::string(".prg");
		filePath = basePath + fileName;
		m_fwriter.open(filePath, std::ios::out);
		//固定格式
		m_fwriter << "//Project Name:   " + fileName << "\n";
		m_fwriter << "//Create Date:   " + Create_time << "\n";
		m_fwriter << "////////////////////////////////////////////////" << "\n";
		m_fwriter << "NOP" << "\n";
        break;
    }
	default:
		break;
	}

    // Export Program Body
    exportCommand(t_cmmdVec);

    // Export Program tail
	switch (t_FileType)
	{
	case ExportFileType::scan_File:
		m_fwriter << "SOCKSEND ID=1 STR=36 B=1" << "\n";
		m_fwriter << "TIMER T=100" << "\n";
		m_fwriter << "SOCKCLOSE ID=1 B=1" << "\n";
		break;
	case ExportFileType::weld_File:
//        m_fwriter << "ARCOFF" << "\n";
		break;
	case ExportFileType::xihu_File:
		m_fwriter << "SOCKCLOSE ID=1 B=1 " << "\n";
		m_fwriter << "TIMER T=100" << "\n";
		m_fwriter << "SOCKOPEN ID=1 TYPE=0 B=1 " << "\n";
		m_fwriter << "TIMER T=100" << "\n";
		m_fwriter << "SOCKSEND ID=1 STR=37 B=1 " << "\n";
		m_fwriter << "TIMER T=100" << "\n";
		m_fwriter << "SOCKCLOSE ID=1 B=1 " << "\n";
		break;
	default:
		break;
	}
	m_fwriter << "END" << "\n";
	m_fwriter.close();

	return true;
}



/// <summary>
/// 转换机器人位置2 固高新坐标
/// </summary>
/// <param name="t_Position"></param>
/// <param name="collaborateMode"></param>
/// <returns></returns>
const GoogolCoord_newVersion FileIO_Utility::convertRobotWaypoint2GoogolCoord_New(const TargetPoint_sptr t_Pnt,
                                                                                  bool collaborateMode)
{
    GoogolCoord_newVersion n_Pose;

//    n_Pose.dynSynch = t_Pnt->getWPPoseData().dynamicTrac;
//    n_Pose.gp_1 = convertRobotPose2GoogolFormat(t_Pnt->getWPPoseData().Pose_Rbt1);
//    n_Pose.gp_2.groupIndex = 2;
//    n_Pose.gp_2 = convertRobotPose2GoogolFormat(t_Pnt->getWPPoseData().Pose_Rbt2);

    return n_Pose;
}

const GroupPose_Googol FileIO_Utility::convertRobotPose2GoogolFormat(const Robot::RobotPose& t_pose)
{
    GroupPose_Googol n_gp;
    n_gp.coordType = (uint)t_pose.CordInfo.first;
    n_gp.configNumber = t_pose.ConfigID;
    n_gp.orientationMode = 1;
    n_gp.posType = 0;
    for(int i = 0; i<t_pose.PoseData.size() ;i++){
        n_gp.joint[0] = t_pose.PoseData[i];
    }

    return n_gp;
}

std::vector<std::string> FileIO_Utility::stdStringSplit(const std::string& srcStr,
                                                        const std::string& delimStr,
                                                        bool repeatedCharIgnored)
{
    std::string tmpSrc(srcStr);
    std::string tmpChar(delimStr);
    std::vector<std::string> resultStringVector;
    std::replace_if(tmpSrc.begin(), tmpSrc.end(), [&](const char& c) {
        if (tmpChar.find(c) != std::string::npos) {
            return true;
        }
        else { return false; }
        }/*pred*/, tmpChar.at(0));
    size_t pos = tmpSrc.find(tmpChar.at(0));
    std::string addedString = "";
    while (pos != std::string::npos) {
        addedString = tmpSrc.substr(0, pos);
        if (!addedString.empty() || !repeatedCharIgnored) {
            resultStringVector.push_back(addedString);
        }
        tmpSrc.erase(tmpSrc.begin(), tmpSrc.begin() + pos + 1);
        pos = tmpSrc.find(tmpChar.at(0));
    }
    addedString = tmpSrc;
    if (!addedString.empty() || !repeatedCharIgnored) {
        resultStringVector.push_back(addedString);
    }
    return resultStringVector;
}

bool FileIO_Utility::exportCommand(const std::vector<RobotCommand_sptr>& commandBuffer)
{
    if(commandBuffer.empty() || !m_fwriter.is_open())
        return false;

    for (auto t_cmmdPtr : commandBuffer) {
        if (t_cmmdPtr->getType() == Robot::CommandType::SetCord) {
            auto coordCommand = static_cast<Robot::CoordCommand*>(t_cmmdPtr.get());
            int coord_ID = coordCommand->getCoordID();
            if (coordCommand->getCoordType() == Robot::CordType::PCS) {
                m_fwriter << "COORD_NUM COOR=PCS ID=" << std::to_string(coord_ID) << "\n";
            }
            else if (coordCommand->getCoordType() == Robot::CordType::MCS) {
                m_fwriter << "COORD_NUM COOR=MCS"<< "\n";
            }
        }
        else if (t_cmmdPtr->getType() == Robot::CommandType::ChgTool){
//            auto toolCommand = static_cast<Robot::ToolCommand*>(t_cmmdPtr.get());
//            int coord_ID = toolCommand->getCoordID();
//            m_fwriter << "COORD_NUM COOR=TCS ID="<< std::to_string(coord_ID) << "\n";

        }
        else if (t_cmmdPtr->getType() == Robot::CommandType::OptTool) {
            auto toolCmmdPtr = static_cast<Robot::ToolCommand*>(t_cmmdPtr.get());
            switch (toolCmmdPtr->getToolType()) {
            case Robot::ToolType::_2DScanner:
                if(toolCmmdPtr->getToolStatus()){
                    m_fwriter << "SOCKCLOSE ID=1 B=1" << "\n";
                    m_fwriter << "TIMER T=250" << "\n";
                    m_fwriter << "SOCKOPEN ID=1 TYPE=0 B=1" << "\n";
                    m_fwriter << "TIMER T=250" << "\n";
                    m_fwriter << "SOCKSEND ID=1 STR=34 B=1" << "\n";
                    m_fwriter << "DOUT DO1.06 = 1" << "\n";
                }
                else{
                    m_fwriter << "SOCKSEND ID=1 STR=35 B=1" << "\n";
                    m_fwriter << "TIMER T=100" << "\n";
                    m_fwriter << "DOUT DO1.06 = 0" << "\n";
                }
                break;
            case Robot::ToolType::WeldTorch:
                if(toolCmmdPtr->getToolStatus()){
                    m_fwriter << "ARCON ID=1" << "\n";
                }
                else{
                    m_fwriter << "ARCOFF ID=1" << "\n";
                }
                break;
            default:
                break;
            }

        }
        else if (t_cmmdPtr->getType() == Robot::CommandType::SetMove) {
            std::string moveType;
            auto movCmmdPtr = static_cast<Robot::MoveCommand*>(t_cmmdPtr.get());
            switch (movCmmdPtr->getMoveType()) {
            case Robot::MoveType::MOVJ:
                if (m_flag_NewVersion)
                    moveType = "MOVJ";
                else
                    moveType = "MOVP";
                break;
            case Robot::MoveType::MOVL:
                moveType = "MOVL";
                break;
            case Robot::MoveType::MOVC:
                moveType = "MOVC";
                break;
            };
            auto moveParam = movCmmdPtr->getMoveParam();
            auto moveSpeed = moveParam[0];
            auto BL = moveParam[1];
            auto VBL = moveParam[2];

            auto t_Waypoint = m_ProgramPtr->getWaypoint_byCommand(t_cmmdPtr);

            auto n_point = convertRobotWaypoint2GoogolCoord_New(t_Waypoint, flag_associatedMove);

            //将gp2的坐标值清空
            n_point.gp_2.coordType = 0;
            n_point.gp_2.configNumber = 0;
            n_point.gp_2.posType = 0;
            n_point.gp_2.orientationMode = 0;


      m_fwriter << moveType << " V=" << std::to_string(moveSpeed)
                            << " BL=" << std::to_string(BL)
                            << " VBL=" << std::to_string(VBL)
                << " @"									//位置型数据开始关键字
                << n_point.gp_1.coordType << ","		//%1		坐标系类型
                << n_point.gp_1.configNumber << ","		//%2		构型
                << n_point.gp_1.posType << ","			//%3		坐标描述模式
                << n_point.gp_1.orientationMode << ","	//%4		姿态描述模式
                << n_point.gp_1.groupIndex << ","		//%5		机器人组号索引
                << 0 << "," << 0 << "," << 0 << ","		//%6~%8		预留位

                << n_point.gp_1.joint[0] << ","
                << n_point.gp_1.joint[1] << ","
                << n_point.gp_1.joint[2] << ","
                << n_point.gp_1.joint[3] << ","
                << n_point.gp_1.joint[4] << ","
                << n_point.gp_1.joint[5] << ","			//%9~14		坐标值 X,Y,Z A,B,C
                << n_point.gp_1.joint[6] << ","
                << n_point.gp_1.joint[7] << "$";			//%14~16	变位机位置


      m_fwriter << n_point.gp_2.coordType << ","
                << n_point.gp_2.configNumber << ","
                << n_point.gp_2.posType << ","
                << n_point.gp_2.orientationMode << ","
                << n_point.gp_2.groupIndex << ","
                << 0 << "," << 0 << "," << 0 << ","

                << n_point.gp_2.joint[0] << ","
                << n_point.gp_2.joint[1] << ","
                << n_point.gp_2.joint[2] << ","
                << n_point.gp_2.joint[3] << ","
                << n_point.gp_2.joint[4] << ","
                << n_point.gp_2.joint[5] << ","
                << n_point.gp_2.joint[6] << ","
                << n_point.gp_2.joint[7] << "$";


      m_fwriter << n_point.extJoint[0] << ","
                << n_point.extJoint[1] << ","
                << n_point.extJoint[2] << ","
                << n_point.extJoint[3] << ","
                << n_point.extJoint[4] << ","
                << n_point.extJoint[5] << ","
                << n_point.extJoint[6] << ","
                << n_point.extJoint[7] << "$";			//外部轴位置

      m_fwriter << n_point.used << ","
                << n_point.dynSynch << ","
                << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";// %43~48 预留位
        }
    }
}

