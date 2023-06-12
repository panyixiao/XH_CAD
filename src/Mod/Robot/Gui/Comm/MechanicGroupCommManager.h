#ifndef ROBOT_MECHANICGROUPCOMMMANAGER_H
#define ROBOT_MECHANICGROUPCOMMMANAGER_H

#include <QObject>
#include <qt5/QtNetwork/qtcpsocket.h>
#include <qt5/QtNetwork/qtcpserver.h>
#include <qt5/QtNetwork/qhostaddress.h>
#include <QList>
#include <QThread>
#include <QTimer>
//#include "Mod/Robot/App/Mechanics/MechanicGroup.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include <functional>

#define SYSTEM_TYPE 2
//#define QT_NO_CAST_FROM_ASCII 1

//typedef std::function<void(const Robot::GroupPose&)> poseUpdateCallBack;

#if (SYSTEM_TYPE == 1)      //OLD SYSTEM

static const char RRT_CMD_SYS_GET[]         = "A";
static const char RRT_CMD_SYS_AUTO_ON[]     = "A2";
static const char RRT_CMD_SYS_AUTO_OFF[]    = "A1";
static const char RRT_CMD_AUX_GET[]         = "A*";

static const char RRT_CMD_IO_GET[]          = "B";
static const char RRT_CMD_IO_SET[]          = "W";

static const char RRT_CMD_POS_GET[]         = "C";  //旧系统机器人运动指令
static const char RRT_CMD_POS_AUTO_ON[]     = "C1";
static const char RRT_CMD_POS_AUTO_OFF[]    = "C2";

static const char RRT_CMD_ALARM_GET[]       = "D";

static const char RRT_CMD_VAR_GET[]         = "E";
static const char RRT_CMD_VAR_SET[]         = "X";

static const char RRT_CMD_SERVO_ON[]        = "Y1";
static const char RRT_CMD_SERVO_OFF[]       = "Y0";
static const char RRT_CMD_SERVO_CLEAR[]     = "Y2";

static const char RRT_CMD_MOTION_START[]    = "Z1";
static const char RRT_CMD_MOTION_PAUSE[]    = "Z2";
static const char RRT_CMD_MOTION_STOP[]     = "Z3";

static const char RRT_CMD_SPEED_SET[]       = "GV"; // should set teaching pad before, refer to manual

static const char RRT_CMD_POS_VAR_GET[]     = "FR";
static const char RRT_CMD_POS_VAR_SET[]     = "FW";

static const char RRT_RESP_HEART_BEAT[]     = "######\r\n";

#elif (SYSTEM_TYPE == 2)    // NEW SYSTEM


static const char RRT_CMD_SYS_GET[]         = "A";
static const char RRT_CMD_SYS_AUTO_ON[]     = "A2";
static const char RRT_CMD_SYS_AUTO_OFF[]    = "A1";
static const char RRT_CMD_AUX_GET[]         = "A*";

static const char RRT_CMD_IO_GET[]          = "B";
static const char RRT_CMD_IO_SET[]          = "W";

static const char RRT_CMD_POS_GET[]         = "{\"Get43\":\"1;0\"}";  //旧系统机器人运动指令
static const char RRT_CMD_POS_AUTO_ON[]     = "C1";
static const char RRT_CMD_POS_AUTO_OFF[]    = "C2";

static const char RRT_CMD_ALARM_GET[]       = "D";

static const char RRT_CMD_VAR_GET[]         = "E";
static const char RRT_CMD_VAR_SET[]         = "X";

static const char RRT_CMD_SERVO_ON[]        = "Y1";
static const char RRT_CMD_SERVO_OFF[]       = "Y0";
static const char RRT_CMD_SERVO_CLEAR[]     = "Y2";

static const char RRT_CMD_MOTION_START[]    = ",{\"Set14\":\"1;1\"},{\"Set3\":\"1;1\"}";
static const char RRT_CMD_MOTION_PAUSE[]    = "Z2";
static const char RRT_CMD_MOTION_STOP[]     = "Z3";

static const char RRT_CMD_SPEED_SET[]       = "GV"; // should set teaching pad before, refer to manual

static const char RRT_CMD_POS_VAR_GET[]     = "FR";
static const char RRT_CMD_POS_VAR_SET[]     = "FW";

static const char RRT_RESP_HEART_BEAT[]     = "######\r\n";

#endif

enum RRT_CMD 
{
	RRT_RC_SYS_GET = 0,
	RRT_RC_POS_GET,
	RRT_RC_VAR_SET,
	RRT_RC_VAR_GET,
    RRT_RC_RUN_PRG,     // 执行示教文件反馈

	RRT_RC_MAX
};

enum RRT_IO_TYPE {
    RRT_IT_DI,
    RRT_IT_DO,
    RRT_IT_AI,
    RRT_IT_AO,
};

typedef struct TAG_ROBOT_INFO{
    double J[14];
    int coord;
    double X;
    double Y;
    double Z;
    double A;
    double B;
    double C;
}RobotInfo;

typedef struct TAG_RRT_IO_INFO {
    int type;
    int channel;
    int bit;
    float value;
} RRT_IO_INFO;

enum RRT_VA_TYPE {
    RRT_VT_B,
    RRT_VT_R,
    RRT_VT_I,
};

typedef struct TAG_RRT_P_INFO {
    int idx;
    float X;
    float Y;
    float Z;
    float A;
    float B;
    float C;
} RRT_P_INFO;

typedef struct TAG_RRT_VA_INFO {
    int type;
    int idx;
    float value;
} RRT_VA_INFO;

namespace Robot{
class MechanicGroup;
// TODO: Abandon QT Thread and QTcpModule, using stl socket modules instead
class ControllerTcpConnector : public QThread
{
    Q_OBJECT
public:
    QTimer reconnectTimer;
    QString robotIp;
    int robotPort;
    // Robot Remote TCP Client
    QTcpSocket *tcpClient;
    bool flag_initialized = false;
    bool flag_stopUpdate = false;

public:
    explicit ControllerTcpConnector(MechanicGroup* t_GroupPtr,
                                    QObject *parent = nullptr);
    ~ControllerTcpConnector(){}

    int  init();
    int  connectToRobot(QString ip, int port);
    int  connectToRobot();
    void disconnectFromRobot();
    bool isRobotConnected();
//    void setPoseUpdateCallBack(poseUpdateCallBack t_func);

    // Mirror Transfer TCP Server
    int  startUpTcpServerMirror(QString ip, int port);
    void shutDownTcpServerMirror();

    void resetFlagCmdRecv(int idx);
    void setFlagCmdRecv(int idx);
    int  getFlagCmdRecv(int idx);

    void resetFlagCmdRecvMirror(int idx);
    void setFlagCmdRecvMirror(int idx);
    int  getFlagCmdRecvMirror(int idx);

    int  flagCmdRecv[RRT_RC_MAX];
    int  flagCmdRecvMirrorClient[RRT_RC_MAX];

    RobotInfo robotCurPos;
    QList<RRT_VA_INFO> listSetVar;
    QList<RRT_VA_INFO> listGetVar;

    void sendCmd(int idx);

    // Get system status info
    int  getSystemStatus();
    // Enable or disable system status auto response while robot state changing
    int  enableSystemStatusAutoResponse(bool enable);
    // Get system auxiliary status
    int  getSystemStatusAux();

    // Get IO status
    int  getIoState(QList<RRT_IO_INFO> list);
    // Set IO status
    int  setIoState(QList<RRT_IO_INFO> list);

    // Get current robot position
    int  getCurrentPosition();
    // Enable or disable auto publish robot position info
    int  enablePositionAutoPublish(bool enable);

    // Get alarm info
    int  getAlarmInfo();

    // Get variable value
    int  getVarVal(QList<RRT_VA_INFO> list);
    // Set variable value
    int  setVarVal(QList<RRT_VA_INFO> list);

    // Enable or disable servo
    int  enableServo(bool enable);
    // Clear alarm
    int  clearServoAlarm();

    // Start motion with teaching file
    int  startMotion(QString file);
    // Pause motion
    int  pauseMotion();
    // Stop motion
    int  stopMotion();

    // Set speed
    int  setSpeed(int speed);

    // Get position type value
    int  getPosVarVal(QList<int> list);
    // Set position type value
    int  setPosVarVal(QList<RRT_P_INFO> list);

    // send heart beat
    int  sendHeartBeat();

    Q_SIGNAL void sigConnectionUpdate(bool s);
    Q_SIGNAL void sigConnectionMirrorUpdate(bool s);
    Q_SIGNAL void sigRobotPoseUpdated();

public Q_SLOTS:
    // Robot Remote TCP Client slots
    void onConnected();
    void onDisconnected();
    void onSocketStateChanged(QAbstractSocket::SocketState socketState);
    void onReadData();
    void onSocketError(QAbstractSocket::SocketError);
    void onReconnectTimer();
    // Mirror Transfer TCP Server slots
    void onNewConnectionMirror();
    void onDisConnectionMirror();
    void onReadDataMirror();
    void onSocketErrorMirror(QAbstractSocket::SocketError e);
protected:
    void run() override;

private:
    MechanicGroup* m_TargetGroup = nullptr;
    MechPose      m_CurrentPose;
//    poseUpdateCallBack    m_PoseUpdateCallBack = nullptr;
    // Mirror Transfer TCP Server
    QTcpServer			tcpServerMirror;
    QList<QTcpSocket*>	tcpServerMirrorConn;

    QString addCmdHeaderTailor(const char* t_cmd);
    void fillCmdChannlBitIO(QString &cmd, RRT_IO_INFO info);
    void fillCmdChannlVA(QString &cmd, RRT_VA_INFO info);
};

}


#endif // ROBOTREMOTETCP_H
