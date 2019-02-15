#include <QDebug>
#include <QTextStream>
#include <QtConcurrent/QtConcurrentRun>
#include <QDateTime>
#include <QtCore>
#include <QJsonDocument>


#include "scrlidar2d.h"

namespace rpl = rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) static_cast<int>(sizeof(_Array) / sizeof(_Array[0]))
#endif


//-----------------------------
SCR::Server::Server(quint16 port, QObject *parent) :
    QObject(parent), m_pWebSocketServer(nullptr){

    m_pWebSocketServer = new QWebSocketServer(QStringLiteral("Websocket Server"),
                                              QWebSocketServer::NonSecureMode,
                                              this);

    m_port= port;

}

//-----------------------------
void SCR::Server::listen(){

    if (m_pWebSocketServer != nullptr){
        if (m_pWebSocketServer->listen(QHostAddress::Any, m_port)){
            qDebug() << "Websocket Server listening on port" << m_port;
            connect(m_pWebSocketServer, &QWebSocketServer::newConnection,
                    this, &SCR::Server::onNewConnection);
        }
    }
}

//-----------------------------
SCR::Server::~Server(){

    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

//-----------------------------
void SCR::Server::onNewConnection(){

    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    qDebug() << "Client connected:" << pSocket->peerName() << pSocket->origin();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &SCR::Server::processRequest);

    connect(pSocket, &QWebSocket::disconnected, this, &SCR::Server::socketDisconnected);

    m_clients << pSocket;
}

//-----------------------------
void SCR::Server::processRequest(QString message){

    qWarning()<<"Received request: "<<message;
    QJsonDocument qjd = QJsonDocument::fromJson(message.toUtf8());

    if (!qjd.isNull()){
        if (qjd.isObject()){
            QJsonObject qjo = qjd.object();
            int cmd = qjo.value("cmd").toInt();
            QString payload = qjo.value("pld").toString();
            if (payload.isEmpty()) emit action(cmd);
            else emit action(cmd, payload);

        } else {
            qWarning()<<"Received text stream is not a json object";
        }
    } else {
        qWarning()<<"Received text stream is not a json document";
    }

    //    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    //    if (pClient)
    //        pClient->sendTextMessage(message);
}

//-----------------------------
void SCR::Server::doResponse(int command, const QString& message){

    qWarning()<<"Response requested for command "<<command;

    for (int ix = 0; ix < m_clients.length(); ++ix){
        m_clients[ix]->sendTextMessage(message);
    }
}

//-----------------------------
void SCR::Server::socketDisconnected(){

    qWarning() << "Client disconnected";
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (pClient){
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

//-----------------------------
SCR::Lidar2d::Lidar2d() {
    drv = nullptr;
    data = nullptr;
    running = false;
    shutdownRequested = false;

}

//-----------------------------
bool SCR::Lidar2d::checkRPLIDARHealth(rpl::RPlidarDriver * drv){
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        qWarning()<<"RPLidar health status :"<<healthinfo.status;
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            qCritical()<<"Error, rplidar internal error detected. Please reboot the device to retry.";
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        qCritical()<<"Error, cannot retrieve the lidar health code:"<< op_result;
        return false;
    }
}

//-----------------------------
int SCR::Lidar2d::createDriver(){
    // create the driver instance
    this->drv = rpl::RPlidarDriver::CreateDriver(rpl::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        qCritical()<<"insufficient memory, exit!";
        return (-2);
    }
    else return (0);
}

//-----------------------------
void SCR::Lidar2d::releaseDriver(){

    if (drv != nullptr){
        rpl::RPlidarDriver::DisposeDriver(drv);
        drv = nullptr;
    }
}

//-----------------------------
void SCR::Lidar2d::shutdown(){

    qWarning()<<"Told to shut down...";
//    this->scandata.lock();
//    this->endscanflag = true;
//    this->scandata.unlock();

    if (drv != nullptr){
        drv->stop();
        drv->stopMotor();
        this->releaseDriver();
    }

    if (data != nullptr){
      delete data;
    };
    shutdownRequested = false;
}

//-----------------------------
void SCR::Lidar2d::doAction(int command, QString payload){

    qWarning()<<"Received action command #"<<command;
    switch(command){
    case 0:{        // status
        concurrent.lock();

        concurrent.unlock();
        break;
    }
    case 1:{        // setup and start
        concurrent.lock();
        if (this->setupAndConnect("/dev/ttyUSB0") >= 0){
            QtConcurrent::run(this, &SCR::Lidar2d::scan);
        }

        concurrent.unlock();
        break;
    }
    case 2: {       // stop
        concurrent.lock();
        shutdownRequested = true;
        concurrent.unlock();
//        this->shutdown();
        break;
    }
    case 3: {       // getdata
        concurrent.lock();

        QString msg = this->getData();
        emit response(command, msg);
        concurrent.unlock();
        break;
    }
    case 4: {       // start logging lidar data
        qWarning()<<"Request to start logging lidar data with tag"<<payload;
        concurrent.lock();
        this->logging = true;
        this->logpath = payload;
        concurrent.unlock();
        break;
    }
    case 5: {
        qWarning()<<"Request to stop logging lidar data";

        concurrent.lock();
        this->logging = false;
        concurrent.unlock();
        break;
    }
    default:{
        break;
    }
    }

}

//-----------------------------
int SCR::Lidar2d::setupAndConnect(QString port){


    if (running){
        qCritical()<<"System is already up and running, don't try yo restart";
        return (-66);
    }
    const char * opt_com_path = nullptr;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    qWarning()<<"SCR LIDAR 2D.\n"<<"Version: "<<RPLIDAR_SDK_VERSION;

    // read serial port from the command line...
    if (! port.isEmpty()) opt_com_path = port.toLatin1().constData(); // or set to a fixed value: e.g. "com3"
    else opt_com_path = "/dev/ttyUSB0";

    opt_com_baudrate = baudrateArray[1];

    int created = this->createDriver();
    if (created != 0) return created;

    // make connection...
    bool connectSuccess = false;
    rplidar_response_device_info_t devinfo;

    if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_OK(op_result)) {
            connectSuccess = true;
        }
        else {
            delete drv;
            drv = nullptr;
        }
    }

    if (!connectSuccess) {
        qCritical()<<"Error, cannot bind to the specified serial port"<<port<<".";
        this->releaseDriver();
        return (-9);
    }


    // print out the device serial number, firmware and hardware version number..

    QString serial = "";

    for (int pos = 0; pos < 16 ;++pos) {
        serial.append(QString::number(devinfo.serialnum[pos], 16));
    }
    qWarning()<<"RPLIDAR S/N:"<<serial;

    qWarning()<<
                 "Firmware Ver:"<<(devinfo.firmware_version>>8)<<(devinfo.firmware_version & 0xFF)<<
                 "Hardware Rev:"<<(static_cast<int>(devinfo.hardware_version));

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        this->releaseDriver();
        return (-8);
    }

    drv->startMotor();

    qWarning()<<"Started Lidar driver, motor spinning";

    this->data = new SCR::Scan();
    qWarning()<<"Scan data initialized, count to zero";

    return 0;
}


//-----------------------------
void SCR::Lidar2d::scan(){
    // fetch result and print it out...
    u_result     op_result;

//    this->scandata.lock();          // lock while we set up...

//    if (this->endscanflag){
//        qWarning()<<"Stop before I started";
//        this->shutdown();
//        return;
//    }
//    stabiliser = new SCR::Stabiliser();

    if (drv == nullptr){
        qWarning()<<"NULLPTR Concurrent run decided it's time to end scan...";
        shutdownRequested = false;
        return;
    }

    drv->startScan(false, true);

//    this->scandata.unlock();

    int ixscan = 0;
    while (true) {
        concurrent.lock();
        if (shutdownRequested){
            qWarning()<<"Client has requested shutdown";
            this->shutdown();
            concurrent.unlock();
            return;
        } else {
            concurrent.unlock();
        }

        ixscan++;
//        qWarning()<<"Lidar2d scanning..."<<ixscan;
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        if (drv == nullptr){
            qWarning()<<"NULLPTR Concurrent run decided it's time to end scan...";
//            this->finishShutdown();
            return;
        }

        op_result = drv->grabScanDataHq(nodes, count);
//        float freq;
//        drv->getFrequency(false, count, &freq);
//        qWarning()<<"Lidar2d full scan complete"<<ixscan;

        if (IS_OK(op_result) || op_result == RESULT_OPERATION_TIMEOUT) {
            drv->ascendScanData(nodes, count);
//            QList<qreal> cdists, cangles, cqs;
            int icount = static_cast<int>(count);

            this->data->clear();
            this->data->count = icount;

            for (int pos = 0; pos < icount ; ++pos) {

                rplidar_response_measurement_node_hq_t node = nodes[pos];
                float distance_meters = node.dist_mm_q2 / 1000.0f / (1<<2);
//                if (distance_meters <= 1e-3f) continue;          // not less than 1 mm
                float angle_degrees = -node.angle_z_q14 * 90.f / (1<<14);

//                qWarning()<<distance_meters;
                float quality = node.quality / 255.0f;
                this->data->dists[pos] = distance_meters;
                this->data->angles[pos] = angle_degrees;
                this->data->qs[pos] = quality;
            }

            if (data->count != 0) emit hasData(this->data);

        } else {
            qWarning()<<"scan op_result is not ok!";
        }
        concurrent.unlock();
//        QThread::msleep(1000/20);     // Give the thread some rest, approx. 20 grabs max per second... What is the real scan speed?
    }
}

//-----------------------------
SCR::Scan *SCR::Lidar2d::getRawData(){

    return  this->data;
}

//-----------------------------
QString SCR::Lidar2d::getData(){

    QString sdata = this->data->serialize();

    return  sdata;
}

//-----------------------------
void SCR::Scan::clear(){

//    this->scandata.lock();

//    qWarning()<<"Clearing data";
    this->count = 0;
//    this->dists.clear();
//    this->angles.clear();
//    this->qs.clear();

//    this->scandata.unlock();
}

//-----------------------------
QString SCR::Scan::serialize(){

    QJsonObject qjo;
    QJsonArray qjaa, qjad, qjaq;

    for (int ix = 0; ix < this->count; ++ix) qjaa.append(static_cast<double>(this->angles[ix]));
    qjo.insert("angles", qjaa);

    for (int ix = 0; ix < this->count; ++ix) qjad.append(static_cast<double>(this->dists[ix]));
    qjo.insert("dists", qjad);

    for (int ix = 0; ix < this->count; ++ix) qjaq.append(static_cast<double>(this->qs[ix]));
    qjo.insert("qs", qjaq);

    qjo.insert("cmd", 3);

    QByteArray qba = QJsonDocument(qjo).toJson(QJsonDocument::Compact);

    return QString(qba);
}


