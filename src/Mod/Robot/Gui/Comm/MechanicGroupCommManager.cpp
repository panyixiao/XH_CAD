

#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "Mod/Robot/App/Mechanics/MechanicGroup.h"
#include "MechanicGroupCommManager.h"
#include "Base/Console.h"

#define interval 200 //ms

using namespace Robot;


ControllerTcpConnector::ControllerTcpConnector(Robot::MechanicGroup *t_GroupPtr,
                                               QObject *parent) :
    QThread(parent)
{
    m_TargetGroup = t_GroupPtr;
    flag_initialized = init() == 0;
}

int ControllerTcpConnector::init()
{
    memset(&robotCurPos, 0, sizeof(robotCurPos));
    memset(flagCmdRecv, 0, RRT_RC_MAX * sizeof(int));
    memset(flagCmdRecvMirrorClient, 0, RRT_RC_MAX * sizeof(int));

    // robot remote control tcp client
    tcpClient = new QTcpSocket(this);
    if (tcpClient == NULL) {
        qDebug() << "[Error] Failed to Initialize TCP Client";
        Base::Console().Error("TCP客户端创建失败\n");
        return -1;
    }

    connect(tcpClient, SIGNAL(connected()), this, SLOT(onConnected()));
    connect(tcpClient, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
    connect(tcpClient, SIGNAL(readyRead()), this, SLOT(onReadData()));
    connect(tcpClient, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(onSocketError(QAbstractSocket::SocketError)));
    connect(&reconnectTimer, SIGNAL(timeout()), this, SLOT(onReconnectTimer()));
    return 0;
}

int ControllerTcpConnector::connectToRobot(QString ip, int port)
{
    robotIp = ip;
    robotPort = port;
    return connectToRobot();
}

int ControllerTcpConnector::connectToRobot()
{
    if (tcpClient == NULL) {
        Base::Console().Error("TCP客户端创建失败\n");
        return -1;
    }
    QHostAddress ipAddr(robotIp);
    tcpClient->connectToHost(ipAddr, robotPort);
    return 0;
}

void ControllerTcpConnector::disconnectFromRobot()
{
    if (tcpClient == NULL) return;
    tcpClient->disconnectFromHost();
}

bool ControllerTcpConnector::isRobotConnected()
{
    if (tcpClient->state() == QAbstractSocket::ConnectedState) {
        return true;
    } else {
        return false;
    }
}


int ControllerTcpConnector::startUpTcpServerMirror(QString ip, int port)
{
    QHostAddress addr;

    if (ip == tr("")) {
        addr = QHostAddress::Any;
    }
    else {
        addr = ip;
    }

    if (!tcpServerMirror.listen(addr, port)) {
        qDebug() << tcpServerMirror.errorString();
        return -1;
    }
    connect(&tcpServerMirror, SIGNAL(newConnection()), this, SLOT(onNewConnectionMirror()));

    qDebug() << "[Info] Robot Remote Control TCP Server Started...";

    return 0;
}

void ControllerTcpConnector::shutDownTcpServerMirror()
{
    disconnect(&tcpServerMirror, SIGNAL(newConnection()), this, SLOT(onNewConnectionMirror()));

    for (int i = 0; i < tcpServerMirrorConn.length(); ++i) {
        tcpServerMirrorConn[i]->disconnectFromHost();
    }
    for (int i = 0; i < tcpServerMirrorConn.length(); ++i) {
        tcpServerMirrorConn[i]->waitForDisconnected();
    }

    tcpServerMirror.close();
}

void ControllerTcpConnector::resetFlagCmdRecv(int idx)
{
    flagCmdRecv[idx] = 0;
}

void ControllerTcpConnector::setFlagCmdRecv(int idx)
{
    flagCmdRecv[idx] = 1;
}

int ControllerTcpConnector::getFlagCmdRecv(int idx)
{
    return flagCmdRecv[idx];
}

void ControllerTcpConnector::resetFlagCmdRecvMirror(int idx)
{
    flagCmdRecvMirrorClient[idx] = 0;
}

void ControllerTcpConnector::setFlagCmdRecvMirror(int idx)
{
    flagCmdRecvMirrorClient[idx] = 1;
}

int ControllerTcpConnector::getFlagCmdRecvMirror(int idx)
{
    return flagCmdRecvMirrorClient[idx];
}

void ControllerTcpConnector::sendCmd(int idx)
{
    switch (idx) {
    case RRT_RC_SYS_GET:
        break;
    case RRT_RC_POS_GET:
        getCurrentPosition();
        break;
    case RRT_RC_VAR_SET:
        setVarVal(listSetVar);
        break;
    case RRT_RC_VAR_GET:
        getVarVal(listGetVar);
    default:
        break;
    }
}

int ControllerTcpConnector::getSystemStatus()
{
    QString cmd = addCmdHeaderTailor(RRT_CMD_SYS_GET);
    tcpClient->write(cmd.toLatin1());
    return 0;
}

int ControllerTcpConnector::enableSystemStatusAutoResponse(bool enable)
{
    QString cmd;

    if (enable) {
        cmd = addCmdHeaderTailor(RRT_CMD_SYS_AUTO_ON);
    } else {
        cmd = addCmdHeaderTailor(RRT_CMD_SYS_AUTO_OFF);
    }
    tcpClient->write(cmd.toLatin1());
    return 0;
}

int ControllerTcpConnector::getSystemStatusAux()
{
    QString cmd = addCmdHeaderTailor(RRT_CMD_AUX_GET);
    auto content = cmd.toLocal8Bit();
    tcpClient->write(content);
    return 0;
}



int ControllerTcpConnector::getCurrentPosition()
{
    QString cmd = addCmdHeaderTailor(RRT_CMD_POS_GET);
    auto content = cmd.toLocal8Bit();
    tcpClient->write(content);
    return 0;
}

int ControllerTcpConnector::getVarVal(QList<RRT_VA_INFO> list)
{
    return 0;
}

int ControllerTcpConnector::setVarVal(QList<RRT_VA_INFO> list)
{
    return 0;
}




int ControllerTcpConnector::startMotion(QString file)
{
#if (SYSTEM_TYPE==1)
    QString cmd = RRT_CMD_MOTION_START;
    cmd += file;
#elif (SYSTEM_TYPE==2)
    QString cmd = tr(RRT_CMD_MOTION_START);
    cmd = tr("{\"Set13\":\"")+file+tr(";1\"}")+cmd;
#endif
    cmd = addCmdHeaderTailor(cmd.toLocal8Bit());
    tcpClient->write(cmd.toLatin1());
    return 0;
}

void ControllerTcpConnector::onConnected()
{
//    qDebug() << "[Info] Connected to robot";
    Base::Console().Message("远程TCP连接机器人控制器成功\n");
    if(m_TargetGroup!=nullptr)
        m_TargetGroup->NetworkConnected.setValue(true);
    Q_EMIT sigConnectionUpdate(true);
}

void ControllerTcpConnector::onDisconnected()
{
//    qDebug() << "[Info] Disconnect from robot done";
    Base::Console().Message("远程TCP连接机器人控制器断开\n");
    if(m_TargetGroup!=nullptr)
        m_TargetGroup->NetworkConnected.setValue(false);
}

void ControllerTcpConnector::onSocketStateChanged(QAbstractSocket::SocketState socketState)
{
    qDebug() << "[Info] onSocketStateChanged" << socketState;
}

void ControllerTcpConnector::onReadData()
{
    int coord = 0;

    QByteArray data = tcpClient->readAll();

    QList<QByteArray> checkout = data.split('\"');

    for (int i = 0; i < tcpServerMirrorConn.length(); ++i) {
        tcpServerMirrorConn[i]->write(data);
    }

    if(checkout.size()<=1)
        return;

    if (checkout[11][0] == 'G' &&
        checkout[11][1] == 'e' &&
        checkout[11][2] == 't' &&
        checkout[11][3] == '4' &&
        checkout[11][4] == '3') {

        QList<QByteArray> dataPackage = data.split('\"');
        QList<QByteArray> poseData = dataPackage[13].split('$');


        if (poseData[1][0] == '1') {
            coord = 1;
        }
        else if (poseData[1][0] == '2') {
            coord = 2;
        }
        else {
            qDebug() << "[Error] Invalid coordinate";
            return;
        }

        poseData[0] = poseData[0].mid(16, (poseData[0].size()-1) - 16);
        poseData[1] = poseData[1].mid(16, (poseData[1].size()-1) - 16);
        poseData[2] = poseData[2].mid(0,poseData[2].size()-1);

        QList<QByteArray> cartPoseData = poseData[0].split(',');
        QList<QByteArray> jontPoseData = poseData[1].split(',');
        QList<QByteArray> exJntPosData = poseData[2].split(',');

        bool ok;
//        m_CurrentPose.Pose_Rbt1.CordInfo = std::make_pair<CordType, int>(CordType::ACS, 0);

//        if(m_CurrentPose.Pose_Rbt1.PoseData.size() == jontPoseData.size()){
//            for(int i = 0; i<jontPoseData.size(); i++){
//                auto d_val = jontPoseData[i].toDouble(&ok);
//                if(ok)
//                    m_CurrentPose.Pose_Rbt1.PoseData[i] = d_val;
//                else{
//                    qDebug() << "[Error] Failed to convert No."<<QString::number(i)<<" AxisVal to double";
//                }
//            }
//        }

//        if(m_CurrentPose.ExtVals.size() == exJntPosData.size()){
//            for(int i = 0; i < exJntPosData.size(); i++){
//                auto d_val = exJntPosData[i].toDouble(&ok);
//                if(ok)
//                    m_CurrentPose.ExtVals[i] = d_val;
//                else{
//                    qDebug() << "[Error] Failed to convert No."<<QString::number(i)<<" ExtVal to double";
//                }
//            }
//        }

        setFlagCmdRecv(RRT_RC_POS_GET);

        /// Bug Record:
        /// QThread will only executing content in func run() in a seprate thread,
        /// onReadData is a custom slot function which will be executed in main thread(FreeCAD thread)
        /// So if you put updating Pose in run(), will leads to a break down bug:
        /// " Fatal IO error 11 (资源暂时不可用) on X server :0.0 "
        /// This bug is usually caused by 2 different threads are trying to take over GUI Resources at same time;
        /// Reference: https://blog.csdn.net/qq_38410730/article/details/80783902
        if(m_TargetGroup!=nullptr)
            m_TargetGroup->setGroupPose(m_CurrentPose);
        return;
    }
}

void ControllerTcpConnector::onSocketError(QAbstractSocket::SocketError error)
{
    qDebug() << "[RemoteTCP] [Error] Failed to connect to robot controller";
    qDebug() << "[RemoteTCP] [Error] Socket Error: " << error;
    Base::Console().Warning("远程TCP连接机器人控制器错误，尝试重新连接\n");
    Q_EMIT sigConnectionUpdate(false);
    //connectToRobot();
    tcpClient->abort();
    reconnectTimer.stop();
    reconnectTimer.setInterval(1000);
    reconnectTimer.start();
}

void ControllerTcpConnector::onReconnectTimer()
{
    connectToRobot();
    reconnectTimer.stop();
}

void ControllerTcpConnector::onNewConnectionMirror()
{
    qDebug() << "[TcpServerMirror] New Connection <---";
    QTcpSocket* conn = tcpServerMirror.nextPendingConnection();
    tcpServerMirrorConn.append(conn);

    connect(conn, SIGNAL(readyRead()), this, SLOT(onReadDataMirror()));
    connect(conn, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(onSocketErrorMirror(QAbstractSocket::SocketError)));
    connect(conn, SIGNAL(disconnected()), this, SLOT(onDisConnectionMirror()));

    Base::Console().Message("轨迹规划控制器连接成功\n");
    Q_EMIT sigConnectionMirrorUpdate(true);
}

void ControllerTcpConnector::onDisConnectionMirror()
{
//	qDebug() << "[TcpServerMirror] Client Disconnected";
//	for (int i = 0; i < tcpServerMirrorConn.length(); ++i) {
//		if (tcpServerMirrorConn[i]->state() == QAbstractSocket::UnconnectedState) {
//			tcpServerMirrorConn[i]->destroyed();
//			tcpServerMirrorConn.removeAt(i);
//		}
//	}

//    if (tcpServerMirrorConn.length() <= 0) {
//        Base::Console().Error("轨迹规划控制器已断开连接");
//        Q_EMIT sigConnectionMirrorUpdate(false);
    //    }
}

void ControllerTcpConnector::onReadDataMirror()
{

}


void ControllerTcpConnector::onSocketErrorMirror(QAbstractSocket::SocketError e)
{
    qDebug() << "[TcpServerMirror] SocketError";
    qDebug() << "[TcpServerMirror] Socket Error: " << e;
    if (tcpServerMirrorConn.length() <= 0) {
        Q_EMIT sigConnectionMirrorUpdate(false);
    }
}

void ControllerTcpConnector::run()
{
    while(!flag_stopUpdate && isRobotConnected()){
        // Request Command
        sendCmd(RRT_RC_POS_GET);
        uint count = 0;
        // Wait For response
        while (!flag_stopUpdate &&
               !getFlagCmdRecv(RRT_RC_POS_GET) &&
               count < 100) {
            msleep(10);
            count++;
        }
        resetFlagCmdRecv(RRT_RC_POS_GET);
        msleep(interval);
    }
}

QString ControllerTcpConnector::addCmdHeaderTailor(const char *t_cmd)
{
    auto cmd = QString::fromLocal8Bit(t_cmd);
#if (SYSTEM_TYPE==1)
    QString buf = "@";
    buf += cmd;
    buf += "&";

#elif (SYSTEM_TYPE==2)
    QString buf = tr("{\"token\":\"googoltech123\",\"post\":[");
    buf += cmd;
    buf += tr("]}");

#endif

    return buf;
}

#include "moc_MechanicGroupCommManager.cpp"


