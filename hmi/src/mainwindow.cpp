#include "mainwindow.h"
#include "datareceiverthread.h"
#include "datapublisherthread.h"
#include "msgprocesstools.h"

#include <iostream>

#include <QLayout>
#include <QDebug>
#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QFont>
#include <QPalette>
using namespace std;
MainWindow::MainWindow(int argc, char** argv, QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow(parent)
{
    Init();
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    if (dataReceiverThread)
    {
        dataReceiverThread->quit();
        dataReceiverThread->wait();
        delete dataReceiverThread;
        dataReceiverThread=nullptr;
    }
    if (threadPublishedStateMsg)
    {
        threadPublishedStateMsg->quit();
        threadPublishedStateMsg->wait();
        delete threadPublishedStateMsg;
        threadPublishedStateMsg=nullptr;
    }
}

void MainWindow::Init()
{
    setObjectName("AIConnect");
    setWindowIcon(QIcon("://source/1.png"));
    setWindowTitle("——————北洋动力——————");

    InitMember();
    InitView();
    LayoutUI();
    InitSlot();

    setWindowFlags(Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
}

void MainWindow::InitMember()
{
  dataReceiverThread = new DataReceiverThread();
  dataReceiverThread->start();

  dataPublisherThread = new DataPublisherThread();
  threadPublishedStateMsg = new QThread();
  dataPublisherThread->moveToThread(threadPublishedStateMsg);

  ui.dockFirst = new QDockWidget(this);
  ui.dockSecond = new QDockWidget(this);
  ui.dockThird = new QDockWidget(this);

  quint16 degree[]={0x2103,0};
  ui.labelMotorTemperature = new QLabel("电机温度");
  ui.labelMotorTemperatureValue = new QLabel("0"+QString::fromUtf16(degree));
  ui.labelControllerTemperature = new QLabel("控制器温度");
  ui.labelControllerTemperatureValue = new QLabel("0"+QString::fromUtf16(degree));
  ui.labelState = new QLabel("无人驾驶任务");
  ui.comboBoxState = new QComboBox();
  ui.toolButtonState = new QToolButton();

  ui.labelCurrentVelocity = new QLabel("当前速度");
  ui.labelCurrentVelocityValue = new QLabel("0");
  ui.labelCurrentVelocityUnit = new QLabel("km/h");
  ui.frameHorizontalLine = new QFrame();

  ui.labelSOC = new QLabel("SOC");
  ui.labelSOCValue = new QLabel("0%");
  ui.labelBatteryTemperature = new QLabel("电池温度");
  ui.labelBatteryTemperatureValue = new QLabel("0"+QString::fromUtf16(degree));
  ui.labelOutputVoltageTemperature = new QLabel("输出电压");
  ui.labelOutputVoltageTemperatureValue = new QLabel("0V");
}

void MainWindow::InitView()
{
  setGeometry(QRect(0,0,1024,300));
  //setGeometry(QApplication::desktop()->availableGeometry());

  QPalette paletteBackgroundGray;
  paletteBackgroundGray.setColor(QPalette::Background, Qt::lightGray);

  QPalette paletteBackgroundBlue;
  paletteBackgroundBlue.setColor(QPalette::Background, QColor("#009ACD"));

  QPalette paletteTextBlue;
  paletteTextBlue.setColor(QPalette::WindowText,QColor("#009ACD"));

  QPalette paletteTextDark;
  paletteTextDark.setColor(QPalette::WindowText,QColor("dark"));

  QPalette paletteTextGray;
  paletteTextGray.setColor(QPalette::WindowText,Qt::lightGray);

  QFont fontNonItalic;
  fontNonItalic.setFamily("Microsoft YaHei");
  fontNonItalic.setPointSize(30);
  fontNonItalic.setWeight(70);
  fontNonItalic.setItalic(false);

  QFont fontItalic;
  fontItalic.setFamily("Microsoft YaHei");
  fontItalic.setPointSize(30);
  fontItalic.setWeight(70);
  fontItalic.setItalic(true);


  {
        ui.labelMotorTemperature->setFont(fontNonItalic);
        ui.labelMotorTemperature->setPalette(paletteTextBlue);
        ui.labelMotorTemperature->setAlignment(Qt::AlignCenter);
        ui.labelMotorTemperatureValue->setFont(fontItalic);
        ui.labelMotorTemperatureValue->setPalette(paletteTextDark);
        ui.labelMotorTemperatureValue->setAlignment(Qt::AlignCenter);
        ui.labelControllerTemperature->setFont(fontNonItalic);
        ui.labelControllerTemperature->setPalette(paletteTextBlue);
        ui.labelControllerTemperature->setAlignment(Qt::AlignCenter);
        ui.labelControllerTemperatureValue->setFont(fontItalic);
        ui.labelControllerTemperatureValue->setPalette(paletteTextDark);
        ui.labelControllerTemperatureValue->setAlignment(Qt::AlignCenter);
        ui.labelState->setAlignment(Qt::AlignCenter);
        QString stringStateStyleSheet = "QLabel {\
                                        font-family: \"Microsoft YaHei\";\
                                        font-size: 30px;\
                                        color: white;\
                                        background-color: #009ACD;}";
        ui.labelState->setStyleSheet(stringStateStyleSheet);
        ui.labelState->setAlignment(Qt::AlignCenter);

        QStringList qstringListComboBoxState;
        qstringListComboBoxState<<"直线加速"<<"八字绕环"<<"高速循迹"<<"EBS"<<"车检"<<"操控性";
        ui.comboBoxState->addItems(qstringListComboBoxState);
        ui.comboBoxState->setCurrentIndex(0);
        ui.comboBoxState->setEditable(true);
        ui.comboBoxState->lineEdit()->setReadOnly(true);
        ui.comboBoxState->lineEdit()->setAlignment(Qt::AlignCenter);


        ui.toolButtonState->setText("任务下发");
        ui.toolButtonState->setMinimumWidth(this->width()*0.2);
        ui.toolButtonState->setAutoRaise(false);

        QWidget* widget = new QWidget();
        QGridLayout* gridLayout = new QGridLayout(widget);
        gridLayout->addWidget(ui.labelMotorTemperature,0,0,1,1);
        gridLayout->addWidget(ui.labelMotorTemperatureValue,1,0,1,1);
        gridLayout->addWidget(ui.labelControllerTemperature,2,0,1,1);
        gridLayout->addWidget(ui.labelControllerTemperatureValue,3,0,1,1);
        gridLayout->addWidget(ui.labelState,4,0,1,1);
        gridLayout->addWidget(ui.comboBoxState,5,0,1,1);
        gridLayout->addWidget(ui.toolButtonState,6,0,1,1);
        ui.dockFirst->setWidget(widget);
        ui.dockFirst->setAutoFillBackground(true);
        ui.dockFirst->setPalette(paletteBackgroundGray);
  }

  {
        ui.labelCurrentVelocity->setFont(fontItalic);
        ui.labelCurrentVelocity->setPalette(paletteTextGray);
        ui.labelCurrentVelocity->setAlignment(Qt::AlignCenter);
        ui.labelCurrentVelocityValue->setFont(fontItalic);
        ui.labelCurrentVelocityValue->setPalette(paletteTextDark);
        ui.labelCurrentVelocityValue->setAlignment(Qt::AlignCenter);
        ui.labelCurrentVelocityUnit->setFont(fontItalic);
        ui.labelCurrentVelocityUnit->setPalette(paletteTextGray);
        ui.labelCurrentVelocityUnit->setAlignment(Qt::AlignCenter);
        ui.frameHorizontalLine->setFrameShape(QFrame::HLine);
        ui.frameHorizontalLine->setLineWidth(10);
        ui.frameHorizontalLine->setPalette(paletteTextGray);

        QWidget* widget = new QWidget();
        QVBoxLayout* vboxLayout = new QVBoxLayout(widget);
        vboxLayout->addStretch();
        vboxLayout->addWidget(ui.labelCurrentVelocity);
        vboxLayout->addWidget(ui.labelCurrentVelocityValue);
        vboxLayout->addWidget(ui.labelCurrentVelocityUnit);
        vboxLayout->addWidget(ui.frameHorizontalLine);
        vboxLayout->addStretch();
        ui.dockSecond->setWidget(widget);
  }

  {
        ui.labelSOC->setFont(fontNonItalic);
        ui.labelSOC->setPalette(paletteTextBlue);
        ui.labelSOC->setAlignment(Qt::AlignCenter);quint16 degree[]={0x2103,0};
        ui.labelSOCValue->setFont(fontItalic);
        ui.labelSOCValue->setPalette(paletteTextDark);
        ui.labelSOCValue->setAlignment(Qt::AlignCenter);
        ui.labelBatteryTemperature->setFont(fontNonItalic);
        ui.labelBatteryTemperature->setPalette(paletteTextBlue);
        ui.labelBatteryTemperature->setAlignment(Qt::AlignCenter);
        ui.labelBatteryTemperatureValue->setFont(fontItalic);
        ui.labelBatteryTemperatureValue->setPalette(paletteTextDark);
        ui.labelBatteryTemperatureValue->setAlignment(Qt::AlignCenter);
        ui.labelOutputVoltageTemperature->setFont(fontNonItalic);
        ui.labelOutputVoltageTemperature->setPalette(paletteTextBlue);
        ui.labelOutputVoltageTemperature->setAlignment(Qt::AlignCenter);
        ui.labelOutputVoltageTemperatureValue->setFont(fontItalic);
        ui.labelOutputVoltageTemperatureValue->setPalette(paletteTextDark);
        ui.labelOutputVoltageTemperatureValue->setAlignment(Qt::AlignCenter);

        QWidget* widget = new QWidget();
        QGridLayout* gridLayout = new QGridLayout(widget);
        gridLayout->addWidget(ui.labelSOC,0,0,1,1);
        gridLayout->addWidget(ui.labelSOCValue,1,0,1,1);
        gridLayout->addWidget(ui.labelBatteryTemperature,2,0,1,1);
        gridLayout->addWidget(ui.labelBatteryTemperatureValue,3,0,1,1);
        gridLayout->addWidget(ui.labelOutputVoltageTemperature,4,0,1,1);
        gridLayout->addWidget(ui.labelOutputVoltageTemperatureValue,5,0,1,1);
        ui.dockThird->setWidget(widget);
        ui.dockThird->setAutoFillBackground(true);
        ui.dockThird->setPalette(paletteBackgroundGray);
  }
}

void MainWindow::InitSlot()
{
  connect(ui.toolButtonState,&QToolButton::clicked,this,&MainWindow::SlotFunctionToolButtonState);
  //告知QObject
  qRegisterMetaType<hmi::StateInterface>("hmi::StateInterface");
  connect(dataReceiverThread,&DataReceiverThread::SIGStateDataReceived,this,&MainWindow::SlotFunctionReceiveStateMsg);
}

void MainWindow::LayoutUI()
{
  //清除自带centeralwidget
  QWidget* p = takeCentralWidget();
  if (p)
  {
    delete p;
    p = nullptr;
  }
  //布局docketwidget
  addDockWidget(Qt::TopDockWidgetArea, ui.dockFirst);
  addDockWidget(Qt::TopDockWidgetArea, ui.dockSecond);
  addDockWidget(Qt::TopDockWidgetArea, ui.dockThird);
  ui.dockFirst->setFeatures(QDockWidget::NoDockWidgetFeatures);
  ui.dockSecond->setFeatures(QDockWidget::NoDockWidgetFeatures);
  ui.dockThird->setFeatures(QDockWidget::NoDockWidgetFeatures);

  QList<QDockWidget*> horizontalFirstRowDockList;
  horizontalFirstRowDockList << ui.dockFirst;
  horizontalFirstRowDockList << ui.dockSecond;
  horizontalFirstRowDockList << ui.dockThird;

  QList<int> horizontalFirstRowSizeList;
  horizontalFirstRowSizeList << static_cast<int>(this->geometry().width() * 0.2);
  horizontalFirstRowSizeList << static_cast<int>(this->geometry().width() * 0.6);
  horizontalFirstRowSizeList << static_cast<int>(this->geometry().width() * 0.2);
  this->resizeDocks(horizontalFirstRowDockList, horizontalFirstRowSizeList, Qt::Horizontal);
}

void MainWindow::SlotFunctionReceiveStateMsg(const hmi::StateInterface stateMsg)
{
  float motorTemperature = stateMsg.Motor_temperature;
  motorTemperature = round(motorTemperature * 100) / 100;

  float controllerTemperature = stateMsg.Controller_temperature;
  controllerTemperature = round(controllerTemperature * 100) / 100;

  int state = stateMsg.State;

  float vehicleSpeed = stateMsg.Vehicle_speed;
  vehicleSpeed = round(vehicleSpeed * 100) / 100;

  int soc = stateMsg.SOC;

  float batteryTemperature = stateMsg.Battery_temp;
  batteryTemperature = round(batteryTemperature * 100) / 100;

  float outputVoltage = stateMsg.Output_voltage;
  outputVoltage = round(outputVoltage * 100) / 100;

  quint16 degree[]={0x2103,0};
  QString stringMotorTemperature = QString::number(motorTemperature)+QString::fromUtf16(degree);
  QString stringControllerTemperature = QString::number(controllerTemperature)+QString::fromUtf16(degree);
  if(mapInt2State.contains(stateMsg.State))
  {
    QString stringState = mapInt2State.value(stateMsg.State);
  }
  else
  {
    QString stringState = "操控性";
  }
  QString stringState = mapInt2State.value(stateMsg.State);
  QString stringVehicleSpeed = QString::number(vehicleSpeed);
  QString stringSOC = QString::number(soc) + "%";
  QString stringBatteryTemperature= QString::number(batteryTemperature)+QString::fromUtf16(degree);
  QString stringOutputVoltage = QString::number(outputVoltage)+"V";

  ui.labelMotorTemperatureValue->setText(stringMotorTemperature);
  ui.labelControllerTemperatureValue->setText(stringControllerTemperature);
  ui.comboBoxState->setCurrentText(stringState);
  ui.labelCurrentVelocityValue->setText(stringVehicleSpeed);
  ui.labelSOCValue->setText(stringSOC);
  ui.labelBatteryTemperatureValue->setText(stringBatteryTemperature);
  ui.labelOutputVoltageTemperatureValue->setText(stringOutputVoltage);
}

void MainWindow::SlotFunctionToolButtonState()
{
  quint16 degree[]={0x2103,0};
  QString stringDegree = QString::fromUtf16(degree);
  QString stringMotorTemperature =ui.labelMotorTemperatureValue->text();
  QString stringControllerTemperature = ui.labelControllerTemperatureValue->text();
  QString stringState = ui.comboBoxState->currentText();
  QString stringVehicleSpeed = ui.labelCurrentVelocityValue->text();
  QString stringSOC = ui.labelSOCValue->text();
  QString stringBatteryTemperature= ui.labelBatteryTemperatureValue->text();
  QString stringOutputVoltage = ui.labelOutputVoltageTemperatureValue->text();
  float motorTemperature = stringMotorTemperature.split(stringDegree)[0].toFloat();
  float controllerTemperature = stringControllerTemperature.split(stringDegree)[0].toFloat();
  float vehicleSpeed = stringVehicleSpeed.toFloat();
  int soc = stringSOC.split("%")[0].toInt();
  float batteryTemperature = stringBatteryTemperature.split(stringDegree)[0].toFloat();
  float outputVoltage = stringOutputVoltage.split(stringDegree)[0].toFloat();
  hmi::StateInterface stateMsg;
  stateMsg.State = mapInt2State.key(stringState,0);
  stateMsg.Battery_temp=batteryTemperature;
  stateMsg.Controller_temperature=controllerTemperature;
  stateMsg.Motor_temperature=motorTemperature;
  stateMsg.Output_voltage=outputVoltage;
  stateMsg.SOC=soc;
  stateMsg.Vehicle_speed=vehicleSpeed;
  dataPublisherThread->SetHmiStaratEndPointMsg(stateMsg);
  connect(threadPublishedStateMsg,&QThread::started,dataPublisherThread,&DataPublisherThread::SlotPublisherHmiStartEndPointMsg,Qt::UniqueConnection);
  if(!threadPublishedStateMsg->isRunning())
  {
    threadPublishedStateMsg->start();
  }
 if(!threadPublishedStateMsg->isRunning())
  {
    QMessageBox::warning(this, "任务下发", "任务下发失败，线程未成功启动!", QMessageBox::Yes, QMessageBox::Yes);
  }
  else
  {
    QMessageBox::information(this, "任务下发", "任务下发成功!", QMessageBox::Yes, QMessageBox::Yes);
  }
}
