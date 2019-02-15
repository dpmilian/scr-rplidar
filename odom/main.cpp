#include <QCoreApplication>
#include "scrlidar2d.h"
#include <QtCore>

int main(int argc, char *argv[]){
    QCoreApplication a(argc, argv);

    SCR::Lidar2d *lidar = new SCR::Lidar2d();
    SCR::Server *server = new SCR::Server(4114);
    server->listen();
    QObject::connect(lidar, &SCR::Lidar2d::finished, &a, &QCoreApplication::quit);
    QObject::connect(server, &SCR::Server::action, lidar, &SCR::Lidar2d::doAction);
    QObject::connect(lidar, &SCR::Lidar2d::response, server, &SCR::Server::doResponse);


    return a.exec();
}
