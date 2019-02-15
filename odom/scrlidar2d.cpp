#include <QDebug>
#include <QTextStream>
#include <QtConcurrent/QtConcurrentRun>
#include <QPixmap>
#include <QDateTime>

#include "scrlidar2d.h"

namespace rpl = rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) static_cast<int>(sizeof(_Array) / sizeof(_Array[0]))
#endif

//-----------------------------
SCR::Lidar2d::Lidar2d() {
    drv = nullptr;
    endscanflag = false;
    stabiliser = nullptr;
    setRenderTarget(QQuickPaintedItem::FramebufferObject);
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
cv::Mat SCR::Lidar2d::buildFrame(qreal arange, int resolution){

    cv::Mat& frame = this->scandata.channels[3];
    frame = cv::Mat::zeros(resolution, resolution, CV_8UC1);

    int l = this->scandata.angles.length();

    cv::Point centre(resolution/2, resolution/2);
    cv::Size axes_size(resolution/2, resolution/2);
    cv::Scalar color(255);

    qreal ratio = static_cast<qreal>(resolution/2 / arange);
    qreal awidth = 2*360.0/l;
    int radius = 0;
//    qWarning()<<"data length:"<<l<<"vs:"<<scandata.dists.length()<<"::"<<scandata.angles.length()<<"::"<<scandata.qs.length();

//    cv::line(frame, centre, cv::Point(resolution/2, 0), color, 4);
    for (int ix = 0; ix < l; ++ix){

        if ((ix > scandata.dists.length())||(ix > scandata.angles.length()))
            qWarning()<<"dists"<<scandata.dists.length()<<", angles"<<scandata.angles.length()<<"vs"<<ix;
        qreal d = scandata.dists[ix];
        qreal a = scandata.angles[ix];

        if (d > range) continue;

        radius = static_cast<int> (ratio * d);

        cv::ellipse(frame, centre, cv::Size(radius, radius), a - 90, -awidth, awidth, color, 1);
    }

    return frame;
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
    this->scandata.lock();
    this->endscanflag = true;
    this->scandata.unlock();

//    if (drv != nullptr){
//        drv->stop();
//        drv->stopMotor();
//        this->releaseDriver();
//    }
}

//-----------------------------
void SCR::Lidar2d::finishShutdown(){

    qWarning()<<"And finish shutting down";
    if (drv != nullptr){
        drv->stop();
        drv->stopMotor();
        this->releaseDriver();
    }
    if (stabiliser != nullptr){
        delete stabiliser;
    }
}

//-----------------------------
int SCR::Lidar2d::setupAndConnect(QString port){


    scandata.channels.push_back(cv::Mat(this->resolution, this->resolution, CV_8UC1, cv::Scalar(10)));
    scandata.channels.push_back(cv::Mat(this->resolution, this->resolution, CV_8UC1, cv::Scalar(170)));
    scandata.channels.push_back(cv::Mat(this->resolution, this->resolution, CV_8UC1, cv::Scalar(10)));
    scandata.channels.push_back(cv::Mat::zeros(this->resolution, this->resolution, CV_8UC1));

    const char * opt_com_path = nullptr;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    qWarning()<<"Ultra simple LIDAR data grabber for RPLIDAR.\n"<<"Version: "<<RPLIDAR_SDK_VERSION;

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
                 "Hardware Rev:"<<((int) devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        this->releaseDriver();
        return (-8);
    }

    drv->startMotor();

    qWarning()<<"Started Lidar driver, motor spinning";
    return 0;
}


//-----------------------------
void SCR::Lidar2d::scan(){
    // fetch result and print it out...
    u_result     op_result;

    this->scandata.lock();          // lock while we set up...

    if (this->endscanflag){
        qWarning()<<"Stop before I started";
        this->finishShutdown();
        return;
    }
    stabiliser = new SCR::Stabiliser();

    if (drv == nullptr){
        qWarning()<<"NULLPTR Concurrent run decided it's time to end scan...";
        return;
    }
    drv->startScan(false, true);

    this->scandata.unlock();


    bool endscan = false;
    int ixscan = 0;
    while (true) {
        ixscan++;
//        qWarning()<<"Lidar2d scanning..."<<ixscan;
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        if (drv == nullptr){
            qWarning()<<"NULLPTR Concurrent run decided it's time to end scan...";
            this->finishShutdown();
            return;
        }

        op_result = drv->grabScanDataHq(nodes, count);
//        float freq;
//        drv->getFrequency(false, count, &freq);
        qWarning()<<"Lidar2d full scan complete"<<ixscan;

        if (IS_OK(op_result) || op_result == RESULT_OPERATION_TIMEOUT) {
            drv->ascendScanData(nodes, count);
//            QList<qreal> cdists, cangles, cqs;
            int icount = static_cast<int>(count);

            this->scandata.lock();

            this->scandata.clear();

            for (int pos = 0; pos < icount ; ++pos) {

                rplidar_response_measurement_node_hq_t node = nodes[pos];
                float distance_meters = node.dist_mm_q2 / 1000.0f / (1<<2);
//                if (distance_meters <= 1e-3f) continue;          // not less than 1 mm
                float angle_degrees = node.angle_z_q14 * 90.f / (1<<14);

//                qWarning()<<distance_meters;
                float quality = node.quality / 255.0f;
                this->scandata.dists.append(static_cast<qreal>(distance_meters));
                this->scandata.angles.append(static_cast<qreal>(angle_degrees));
                this->scandata.qs.append(static_cast<qreal>(quality));
            }
//            qWarning()<<"Parsed all scan data"<<ixscan;

            endscan = this->endscanflag;

            if (!endscan){
                cv::Mat frame = this->buildFrame(this->range, this->resolution);
                stabiliser->step(frame);
            }

            if (endscan){
//                qWarning()<<"Concurrent run decided it's time to end scan...";
                this->scandata.unlock();
                this->finishShutdown();
                return;
            }

            if (scandata.dists.length() != 0) emit hasData();

            this->scandata.unlock();

        } else {
            qWarning()<<"scan op_result is not ok!";
        }
    }
}


//-----------------------------
void SCR::Lidar2d::runLidar(){
    // start concurrent scan thread...
    connect (this, &SCR::Lidar2d::concurrentFinished, this, &SCR::Lidar2d::finishShutdown);
    QtConcurrent::run(this, &SCR::Lidar2d::scan);

}


//-----------------------------
QList<qreal> SCR::Lidar2d::getData(char what){

    this->scandata.lock();

    QList<qreal> retval;

    switch(what){
    case 0:{
        retval = this->scandata.dists;
//        retval<<this->scandata.dists;
//        retval<<this->scandata.angles;
//        retval<<this->scandata.qs;
        break;
    }
    case 1:{
        retval = this->scandata.angles;
        break;
    }
    case 2:{
        retval = this->scandata.qs;
        break;
    }
    }

    this->scandata.unlock();

    return retval;

}

//-----------------------------
void SCR::ScanData::clear(){

//    this->scandata.lock();

    qWarning()<<"Clearing data";
    this->dists.clear();
    this->angles.clear();
    this->qs.clear();

//    this->scandata.unlock();
}

//-----------------------------
QPixmap SCR::Lidar2d::getFrame(){

    qWarning()<<"Getting frame";

    this->scandata.lock();

    qWarning()<<"Got into getting frame";
    cv::Mat frame;
    cv::merge(this->scandata.channels, frame);

//    cv::imwrite("/home/dani/Pictures/LIDAR/" + QString::number(QDateTime::currentMSecsSinceEpoch()).toStdString() + ".png", frame);
    QImage image(frame.data,
                 frame.rows, frame.cols,
                 static_cast<int>(frame.step),
                 QImage::Format_RGBA8888/*Grayscale8*/);

    this->scandata.unlock();

    return QPixmap::fromImage(image);

}

//-----------------------------
void SCR::Lidar2d::paint(QPainter* painter) {

    qWarning()<<"Paint requested on Lidar2D";
    QPixmap qpix = this->getFrame();
    painter->drawPixmap(0,0, qpix);
    qWarning()<<"Paint finished";

}
