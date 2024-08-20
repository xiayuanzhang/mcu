#ifndef COMMFRAME_H
#define COMMFRAME_H

/**
 * 因为TCP通信存在粘包的可能性, 比如连续发送前进和后退两条指令, 单片机可能收到是两条指令合在一起的数据, 导致无法正确解析.
 * 因此添加通信协议, 单片机收到之后, 就可以解析出两帧数据
通信协议定义

0xAA ID LEN VALUE 0x55

0xAA为固定的帧头(uint8), ID为帧ID(uint8), LEN为帧长度(uint8), VALUE为帧数据(类型不定,根据ID解析),0x55为固定的帧尾(uint8)

控制电机前进
ID = 0x01 LEN = 0

控制电机后退
ID = 0x02 LEN = 0

控制电机左转
ID = 0x03 LEN = 0

控制电机右转
ID = 0x04 LEN = 0

控制电机停止
ID = 0x05 LEN = 0

控制电机速度
ID = 0x06 LEN = 1 VALUE = 0~255

控制继电器,0关闭,1打开
ID = 0x07 LEN = 1 VALUE = 0/1

传感器数据
ID = 0x10 LEN =  16 VALUE = flaot类型的温度 + float类型的湿度 + float类型的可燃气体浓度 + float类型的烟雾浓度
*/


#include <QObject>
#include <QTimer>
#include <QVector>

class CommFrame : public QObject
{
    Q_OBJECT
public:
    explicit CommFrame(QObject *parent = nullptr);

    void pushData(QByteArray data);

    QByteArray packFrame(uint8_t id);
    QByteArray packFrame(uint8_t id, uint8_t value);
    QByteArray packFrame(uint8_t id, QVector<uint8_t> value);
signals:
    void haveFrame(uint8_t id, QVector<uint8_t> value);

private:
    void parseFrame();
private:
    const int queueMaxLen = 10*1024;//10K
    QVector<uint8_t> queue;
    QTimer *timer;


};

#endif // COMMFRAME_H
