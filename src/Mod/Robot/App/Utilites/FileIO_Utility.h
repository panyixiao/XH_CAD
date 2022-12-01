#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "DS_Utility.h"
#include <Mod/Robot/App/Trac/RobotProgram.h>

enum class ExportFileType {
	scan_File = 0,
	weld_File,
	xihu_File,
	test_File
};

// Old Version of Controller
struct GoogolCoord_oldVersion {
	int pntID = { 0 };
	float J1 = { 0 };
	float J2 = { 0 };
	float J3 = { 0 };
	float J4 = { 0 };
	float J5 = { 0 };
	float J6 = { 0 };
	float J7 = { 0 };
	float J8 = { 0 };
	int pose = { 0 };
};

// New Version of Controller
struct GroupPose_Googol
{
	short coordType = { 1 };        // 坐标系类型 0=PCS 工件坐标系 1=MCS 直角坐标系 2=ACS 关节坐标系 3=TCS 工具坐标系 4=VCS 速度坐标系
	short configNumber = { 1 };     // 构型选择 ACS下运动时，设置为1
	short posType = { 0 };          // 坐标描述模式 0=绝对位置  1=增量型位置
	short orientationMode = { 1 };  // 姿态描述模式 0=无姿态描述 1=ZYX欧拉角姿态描述 2=四元数姿态描述 3=刀轴矢量姿态描述 4=旋转轴位置姿态描述
	short groupIndex = { 0 };       // 机器人组号索引 仅对机器人组2有效，与【变位系统】中标定的机器人组号对应。
	short rev[3] = { 0 };           // 预留位，暂不使用
    double joint[8] = { 0.0 };      // 坐标值
};

struct GoogolCoord_newVersion {
	int pntID = { 0 };
    GroupPose_Googol gp_1;
    GroupPose_Googol gp_2;
	double extJoint[8] = { 0.0 };
    bool used = { true };      // 位置点被标定后，将该标志置为1，表示位置点有效；MOVL、MOVJ等指令的运动目标若为“无效点”，译码模块报错。
	bool dynSynch = { false }; // 机器人组1与机器人组2存在运动学插补联动关系时，将该标志置为1，表示动态跟踪使能
};

class FileIO_Utility
{
public:
    FileIO_Utility();
    bool generateProgramSequence(const std::string& t_filePath,
                                 std::shared_ptr<Robot::RobotProgram> t_bufferPtr);

	void setExportPara_NewVersion(bool isTrue) {
		m_flag_NewVersion = isTrue;
	}
	void setExportPara_AssociateMove(bool isTrue) {
		flag_associatedMove = isTrue;
	}
    bool splitProgramBySequence();

	void setScanToolID(const int id) {
		m_scanToolID = id;
	}
	void setWeldToolID(const int id) {
		m_weldToolID = id;
	}
	void enableSpeedSetting(const bool flag) {
		flag_enableSpeedSetting = flag;
	}

    void setCustomSpeed(int type, float speed) {
		switch (type) {
		case 0:
			m_scanSpeed = speed;
			break;
		case 1:
			m_moveSpeed = speed;
			break;
		case 2:
			m_nearSpeed = speed;
			break;
		case 3:
			m_Jspeed = speed;
			break;
		}
	}
	void setFlagSplit(bool splitFile) {
		m_flag_Split = splitFile;
	}

    static std::shared_ptr<Robot::RobotProgram> readinProgramFromDrive(const std::string& t_filePath, const string &t_executror);

private:
    int toolIDtranslate(const int t_ToolID);
    bool progTranslator_old(const ExportFileType& t_Type,
                            const std::string basePath,
                            const int sequenceID,
                            const std::vector<RobotCommand_sptr>& t_cmds);

    bool progTranslator_new(const ExportFileType& t_Type,
                            const std::string basePath,
                            const int sequenceID,
                            const std::vector<RobotCommand_sptr>& t_cmds);

    const GoogolCoord_newVersion convertRobotWaypoint2GoogolCoord_New(const RobotWaypoint_sptr t_Pnt,
                                                                      bool collaborateMode = false);
    const GroupPose_Googol convertRobotPose2GoogolFormat(const Robot::RobotPose &t_pose);
    static std::vector<std::string> stdStringSplit(const std::string& srcStr,
                                                   const std::string& delimStr,
                                                   bool repeatedCharIgnored);

    bool exportCommand(const std::vector<RobotCommand_sptr> &commandBuffer);


private:
    std::shared_ptr<Robot::RobotProgram> m_ProgramPtr = nullptr;
    std::vector<std::pair<int, std::vector<RobotCommand_sptr>>> m_Vec_JOIN;
    std::vector<std::pair<int, std::vector<RobotCommand_sptr>>> m_Vec_SCAN;
    std::vector<std::pair<int, std::vector<RobotCommand_sptr>>> m_Vec_WELD;

	std::ofstream m_fwriter;
	bool flag_enableSpeedSetting = false;
    float m_scanSpeed = 0.0;
    float m_moveSpeed = 0.0;
    float m_nearSpeed = 0.0;
    int m_Jspeed = 0;
    int m_scanToolID = 1;
    int m_weldToolID = 2;
    bool m_flag_Split = false;
    bool m_flag_NewVersion = true;
    bool flag_associatedMove = false;
};


