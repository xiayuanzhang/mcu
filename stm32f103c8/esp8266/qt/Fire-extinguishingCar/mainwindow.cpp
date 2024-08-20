#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    csocket = new QTcpSocket(this);
    connect(csocket, &QTcpSocket::readyRead, this, &MainWindow::receiveData);
    connect(csocket, &QTcpSocket::connected, this, &MainWindow::tcpConnect);
    connect(csocket, &QTcpSocket::disconnected, this, &MainWindow::tcpDisconnect);

    commFrame = new CommFrame(this);
    connect(commFrame, &CommFrame::haveFrame, this, &MainWindow::parseFrame);


    configIni = new QSettings(QApplication::applicationDirPath()+"config.ini", QSettings::IniFormat);
    configIni->setIniCodec("UTF-8");

    //读取配置文件
    QString ip = configIni->value("IP").toString();
    QString port = configIni->value("PORT").toString();
    ui->IPNum->setText(ip);
    ui->PORTNum->setText(port);

    connectStatus = new QLabel("服务器状态: 未连接");
    ui->statusbar->addPermanentWidget(connectStatus,0);
}

MainWindow::~MainWindow()
{
    if(csocket->state() == QAbstractSocket::ConnectedState){
        csocket->disconnectFromHost();
    }
    delete ui;
}

//连接服务器
void MainWindow::on_connectBtn_clicked(){
    // 根据输入的ip和port连接指定的服务器
    qDebug()<<"clicked";

    configIni->setValue("IP",ui->IPNum->text());
    configIni->setValue("PORT",ui->PORTNum->text());

    if(csocket->state() != QAbstractSocket::ConnectedState){
        qDebug()<<ui->IPNum->text()<< ui->PORTNum->text().toUShort();
        csocket->connectToHost(ui->IPNum->text(), ui->PORTNum->text().toUShort());
    }
}

void MainWindow::on_disconnectBtn_clicked()
{
    if(csocket->state() == QAbstractSocket::ConnectedState){
        csocket->disconnectFromHost();
    }
}


//获取服务器发送的数据
void MainWindow::receiveData()
{
    QByteArray data = csocket->readAll();
    qDebug()<<data;
    commFrame->pushData(data);
}

void MainWindow::tcpConnect()
{
    qDebug()<<"is connect";
    connectStatus->setText("服务器状态: 已连接");
}

void MainWindow::tcpDisconnect()
{
    qDebug()<<"is disconnect";
    connectStatus->setText("服务器状态: 未连接");
    if (csocket->error() == QAbstractSocket::RemoteHostClosedError) {
        qDebug() << "Server closed the connection";
    } else if (csocket->error() != QAbstractSocket::SocketError::UnknownSocketError) {
        qDebug() << "Error:" << csocket->errorString();
    } else {
        qDebug() << "Disconnected from server";
    }
}

void MainWindow::parseFrame(uint8_t id, QVector<uint8_t> value)
{
    if(id == 0x10){
        if(value.size() != 16){
            return;
        }
        float temp = 0;
        memcpy(&temp, value.data(), 4);
        ui->TemperNum->setText(QString::number(temp));
        memcpy(&temp, value.data()+4, 4);
        ui->MoisNum->setText(QString::number(temp));
        memcpy(&temp, value.data()+8, 4);
        ui->GasNum->setText(QString::number(temp));
        memcpy(&temp, value.data()+12, 4);
        ui->SmokeNum->setText(QString::number(temp));
    }
}

//小车控制发送数据
//前
void MainWindow::on_Forward_clicked()
{
    QByteArray data = commFrame->packFrame(0x01);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}
//后
void MainWindow::on_Back_clicked()
{
    QByteArray data = commFrame->packFrame(0x02);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}
//左
void MainWindow::on_Left_clicked()
{
    QByteArray data = commFrame->packFrame(0x03);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}
//右
void MainWindow::on_Right_clicked()
{
    QByteArray data = commFrame->packFrame(0x04);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}
//停
void MainWindow::on_Stop_clicked()
{
    QByteArray data = commFrame->packFrame(0x05);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}





//关闭灭火器
void MainWindow::on_pushButton_2_clicked()
{
    QByteArray data = commFrame->packFrame(0x07,0);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}

//开启灭火器
void MainWindow::on_pushButton_clicked()
{
    QByteArray data = commFrame->packFrame(0x07,1);

    // 发送文本数据
    if (csocket->state() == QAbstractSocket::ConnectedState) {
        csocket->write(data);
    } else {
        qDebug() << "Error: Socket not open or not connected";
        return;
    }
}

