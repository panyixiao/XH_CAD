/****************************************



****************************************/
while (!bThreadExit) {
      
     if (estop) {
        stopRecord();
        break;
    }

    // get start pos from robot
    emit sigSendCmdRRT(RRT_RC_POS_GET);                                          //发送姿态获取请求
    while (!bThreadExit && !robotRemoteTcp->getFlagCmdRecv(RRT_RC_POS_GET)) {
        msleep(1);
        if (estop) break;
    }

    robotRemoteTcp->resetFlagCmdRecv(RRT_RC_POS_GET);

    RobotInfo r = robotRemoteTcp->robotCurPos;
    qint64 timestamp = GetSysTimeMicros();
    RobotPosStamped pos;
    pos.robotPos = r;
    pos.timestamp = timestamp;
    robotPos.push_back(pos);

    J7=r.J[6];
    J8=r.J[7];
    J9=r.J[8];

    // exit scanning when receive stop mvoe
    if (robotCommandTcp->getFlagCmdRecv(RRTT_CMD_STOP_MOVE)) {
        robotCommandTcp->resetFlagCmdRecv(RRTT_CMD_STOP_MOVE);
        qDebug() << "[Info] Received STOP MOVE";
        Log4Info(QString::fromLocal8Bit("接收到机器人控制器结束扫描指令"));
        stopRecord();
        break;
    }
    msleep(1);
 
    }

connect(this, SIGNAL(sigSendCmdRRT(int)), this, SLOT(onSendCmdRRT(int)));

void WeldTrack::onSendCmdRRT(int idx)
{
    robotRemoteTcp->sendCmd(idx);
}

void RobotRemoteTcp::sendCmd(int idx)
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

int RobotRemoteTcp::getCurrentPosition()
{
    QString cmd = addCmdHeaderTailor(RRT_CMD_POS_GET);
    tcpClient->write(cmd.toLatin1());
    return 0;
}

static const char RRT_CMD_POS_GET[]= "{\"Get43\":\"1;0\"}";  //机器人运动指令

QString RobotRemoteTcp::addCmdHeaderTailor(QString cmd)
{
#if (SYSTEM_TYPE==1)
    QString buf = "@";
    buf += cmd;
    buf += "&";

#elif (SYSTEM_TYPE==2)
    QString buf = "{\"token\":\"googoltech123\",\"post\":[";
    buf += cmd;
    buf += "]}";
#endif
    return buf;
}
