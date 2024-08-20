#include "commframe.h"

CommFrame::CommFrame(QObject *parent)
    : QObject{parent}
{
    timer = new QTimer{this};
    connect(timer, &QTimer::timeout, this, &CommFrame::parseFrame);
    timer->start(10);//10ms
}

void CommFrame::pushData(QByteArray data)
{
    if(queue.size() < queueMaxLen){
        QVector<uint8_t> temp;
        for(auto i : data){
            temp.push_back(i);
        }
        queue.append(temp);
    }
}

QByteArray CommFrame::packFrame(uint8_t id)
{
    QByteArray frame;
    frame.append(0xAA);
    frame.append(id);
    frame.append('\0');
    frame.append(0x55);
    return frame;
}

QByteArray CommFrame::packFrame(uint8_t id, uint8_t value)
{
    QByteArray frame;
    frame.append(0xAA);
    frame.append(id);
    frame.append(1);
    frame.append(value);
    frame.append(0x55);
    return frame;
}

QByteArray CommFrame::packFrame(uint8_t id, QVector<uint8_t> value)
{
    QByteArray frame;
    frame.append(0xAA);
    frame.append(id);
    frame.append(value.size());
    for(auto i : value){
        frame.append(i);
    }
    frame.append(0x55);
    return frame;
}

void CommFrame::parseFrame()
{
    while(queue.size() > 0){
        int index = queue.indexOf(0xAA);
        if(index == -1){
            queue.clear();
            return;
        }

        //去掉0xAA之前的数据, 保证0xAA在第一位
        if(index > 0){
            queue = queue.mid(index);
        }
        //校验数据长度
        if(queue.size() < 4){
            return;
        }
        //校验数据长度
        if(queue.size() < queue[2] + 4){
            return;
        }

        //校验帧尾,
        if(queue[queue[2] + 3] != 0x55){
            //向后偏移一个字节
            queue = queue.mid(1);
            continue;
        }

        emit haveFrame(queue[1], queue.mid(3, queue[2]));

        queue = queue.mid(queue[2] + 4);
    }
}
