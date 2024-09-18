#include "datapublisherthread.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>

DataPublisherThread::DataPublisherThread(QObject* parent)
    :QObject(parent)
    ,hmiStartEndPointePub()
{

}

DataPublisherThread::~DataPublisherThread()
{
    ros::shutdown();
}


void DataPublisherThread::SetHmiStaratEndPointMsg(const hmi::StateInterface msg)
{
    hmiStartEndPointMsg = msg;
}


void DataPublisherThread::SlotPublisherHmiStartEndPointMsg()
{
    if(hmiStartEndPointePub==ros::Publisher())
    {
        nh= ros::NodeHandle();
        hmiStartEndPointePub= nh.advertise<hmi::StateInterface>("state_data", 1000);
    }

    // 定义循环发送消息的逻辑
    ros::Rate loop_rate(10);  // 假设每秒发送10条消息
    while (ros::ok())
    {

         // 发布消息
         hmiStartEndPointePub.publish(hmiStartEndPointMsg);

         // 处理ROS回调
         ros::spinOnce();

         // 休眠一段时间，以控制发送频率
         loop_rate.sleep();
    }
}
