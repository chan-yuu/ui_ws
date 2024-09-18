
#ifndef DATAPUBLISHERTHREAD_H
#define DATAPUBLISHERTHREAD_H

#include <ros/ros.h>
#include <QString>
#include <QObject>

#include "hmi/StateInterface.h"
class DataPublisherThread : public QObject
{
    Q_OBJECT

public:
    explicit DataPublisherThread(QObject* parent = nullptr);
    ~DataPublisherThread();
public:
    void SetHmiStaratEndPointMsg(const hmi::StateInterface msg);
public slots:
    void SlotPublisherHmiStartEndPointMsg();
protected:
private:

private:
    ros::NodeHandle nh;

    ros::Publisher hmiStartEndPointePub;
    hmi::StateInterface hmiStartEndPointMsg;



};

#endif // DATAPUBLISHERTHREAD_H

