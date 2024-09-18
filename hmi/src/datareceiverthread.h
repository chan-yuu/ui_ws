#ifndef DATARECEIVERTHREAD_H
#define DATARECEIVERTHREAD_H
#include <QCoreApplication>
#include <QThread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "hmi/StateInterface.h"

class DataReceiverThread : public QThread
{
    Q_OBJECT
public:
    DataReceiverThread();
    ~DataReceiverThread();

signals:
    void SIGStateDataReceived(const hmi::StateInterface);

protected:
    void run() override;


private slots:
    void stateCallback(const hmi::StateInterface msg);


private:
    hmi::StateInterface stateMsg;

};
#endif // DATARECEIVERTHREAD_H
