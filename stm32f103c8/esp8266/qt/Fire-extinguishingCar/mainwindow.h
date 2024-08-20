#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QFileDialog>
#include <QSettings>
#include "commframe.h"
#include <QLabel>


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void receiveData();
    void tcpConnect();
    void tcpDisconnect();
    void parseFrame(uint8_t id, QVector<uint8_t> value);
private slots:
    void on_connectBtn_clicked();
    void on_Forward_clicked();
    void on_Left_clicked();
    void on_Right_clicked();
    void on_Back_clicked();
    void on_Stop_clicked();

    void on_disconnectBtn_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QLabel *connectStatus;
    //客户端的套接字
    QTcpSocket  *csocket;
    //通信协议
    CommFrame *commFrame;

    QSettings *configIni;
};
#endif // MAINWINDOW_H
