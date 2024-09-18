#include "datareceiverthread.h"
DataReceiverThread::DataReceiverThread()
{

}

DataReceiverThread::~DataReceiverThread()
{

}

void DataReceiverThread::run()
{
    // Initialize ROS node
    ros::NodeHandle nh;

    // Create ROS subscribers
    ros::Subscriber stateSub = nh.subscribe("state_data", 10, &DataReceiverThread::stateCallback, this);

    // Spin ROS node
    ros::spin();

}

void DataReceiverThread::stateCallback(const hmi::StateInterface msg)
{
    emit SIGStateDataReceived(msg);
}
