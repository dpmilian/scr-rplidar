#ifndef SCRLIDAR2D_H
#define SCRLIDAR2D_H

//#include <opencv2/opencv.hpp>
#include "rplidar/include/rplidar.h"
//#include <QString>
#include <QVector>
#include <QObject>
#include <QtWebSockets/QWebSocketServer>
#include <QtWebSockets/QWebSocket>
#include <QMutex>

namespace rpl = rp::standalone::rplidar;

namespace SCR{

class Server : public QObject{
    Q_OBJECT
public:
    explicit Server(quint16 port, QObject *parent = nullptr);
    ~Server() override;    
    void listen();

signals:
    void action(int command, QString payload="");

public Q_SLOTS:
    void doResponse(int command, const QString& message = "");

private Q_SLOTS:
    void onNewConnection();
    void processRequest(QString message);
    void socketDisconnected();

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    quint16 m_port;

};



class Scan {

public:
    Scan(){this->dists.resize(4096); this->angles.resize(4096); this->qs.resize(4096);this->count = 0;}
    QVector<float> dists, angles, qs;
    int count;
//    std::vector<cv::Mat> channels;

//    void lock(){ _lock.lock();}
//    void unlock(){_lock.unlock();}

    void clear();
    QString serialize();
private:
//    QMutex _lock;

};

class Lidar2d : public QObject {
    Q_OBJECT

public:
    Lidar2d();
    /*Q_INVOKABLE */int setupAndConnect(QString port = "");
//    void paint(QPainter* painter) override;
//    Q_PROPERTY (ScanData scandata)

    QString getData();

signals:
    void hasData(SCR::Scan* data);
    void finished();
    void response(int command, const QString& msg);

public slots:
    void scan();
    SCR::Scan* getRawData();
    void shutdown();
//    void finishShutdown();
    void setRange(float arange){this->range = arange;}
    void setResolution(int aresolution){this->resolution = aresolution;}
    void doAction(int command, QString payload);

private:
    bool checkRPLIDARHealth(rpl::RPlidarDriver *drv);
    void releaseDriver();
    int createDriver();
//    cv::Mat buildFrame(float range, int resolution);


    rpl::RPlidarDriver *drv;
    QMutex concurrent;
    bool running, logging, shutdownRequested;
    QString logpath;

//    bool endscanflag;
//    SCR::Stabiliser* stabiliser;
    float range;
    int resolution;
    SCR::Scan* data;
};
}


#endif // SCRLIDAR2D_H
