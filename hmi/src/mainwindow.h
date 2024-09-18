#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QDockWidget>
#include <QLabel>
#include <QComboBox>
#include <QToolButton>
#include <QLineEdit>
#include <QFrame>
#include <QThread>
#include <QMetaType>
#include <sensor_msgs/Image.h>
#include "hmi/StateInterface.h"

class DataPublisherThread;
class DataReceiverThread;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget* parents = nullptr, Qt::WindowFlags flags = 0);
    ~MainWindow();
public:
    void Init();
protected:
    void InitMember();
    void InitView();
    void InitSlot();
    void LayoutUI();
private slots:
    void SlotFunctionReceiveStateMsg(const hmi::StateInterface stateMsg);
    void SlotFunctionToolButtonState();
private:
    DataReceiverThread* dataReceiverThread;
    DataPublisherThread* dataPublisherThread;
    QThread* threadPublishedStateMsg;

    struct UIMainWindow
    {
        /*-----Layouts-------- */
        //主界面左侧
        QDockWidget* dockFirst;
        //主界面中间
        QDockWidget* dockSecond;
        //主界面右侧
        QDockWidget* dockThird;


        /*-----Widgets-------- */
        QLabel* labelMotorTemperature;
        QLabel* labelMotorTemperatureValue;
        QLabel* labelControllerTemperature;
        QLabel* labelControllerTemperatureValue;
        QLabel* labelState;
        QComboBox* comboBoxState;
        QToolButton* toolButtonState;

        QLabel* labelCurrentVelocity;
        QLabel* labelCurrentVelocityValue;
        QLabel* labelCurrentVelocityUnit;
        QFrame* frameHorizontalLine;

        QLabel* labelSOC;
        QLabel* labelSOCValue;
        QLabel* labelBatteryTemperature;
        QLabel* labelBatteryTemperatureValue;
        QLabel* labelOutputVoltageTemperature;
        QLabel* labelOutputVoltageTemperatureValue;
    };

    UIMainWindow ui;
    const QMap<int,QString> mapInt2State = {
        {1,"直线加速"},
        {2,"八字绕环"},
        {3,"高速循迹"},
        {4,"EBS"},
        {5,"车检"},
        {6,"操控性"},

    };
};
#endif // MAINWINDOW_H
