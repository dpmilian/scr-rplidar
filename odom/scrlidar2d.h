#ifndef SCRLIDAR2D_H
#define SCRLIDAR2D_H

#include <opencv2/opencv.hpp>
#include "rplidar/include/rplidar.h"
#include <QObject>
#include "stabiliser.h"
//#include <QString>
#include <QList>
#include <QMutex>
#include <QQuickPaintedItem>
#include <QPainter>

namespace rpl = rp::standalone::rplidar;

namespace SCR{

class ScanData {

public:
    QList<qreal> dists, angles, qs;
    std::vector<cv::Mat> channels;

    void lock(){ _lock.lock();}
    void unlock(){_lock.unlock();}

    void clear();
private:
    QMutex _lock;

};

class Lidar2d : public QQuickPaintedItem {
    Q_OBJECT

public:
    Lidar2d();
    Q_INVOKABLE int setupAndConnect(QString port = "");
    void paint(QPainter* painter) override;
//    Q_PROPERTY (ScanData scandata)


signals:
    void hasData();
    void concurrentFinished();

public slots:
    void runLidar();
    QList<qreal> getData(char what);
    QPixmap getFrame();
    void shutdown();
    void finishShutdown();
    void setRange(qreal arange){this->range = arange;}
    void setResolution(int aresolution){this->resolution = aresolution;}

private:
    bool checkRPLIDARHealth(rpl::RPlidarDriver *drv);
    void releaseDriver();
    int createDriver();
    cv::Mat buildFrame(qreal range, int resolution);



    rpl::RPlidarDriver *drv;
    void scan();

    bool endscanflag;
    SCR::Stabiliser* stabiliser;
    qreal range;
    int resolution;
    SCR::ScanData scandata;
};
}


#endif // SCRLIDAR2D_H
