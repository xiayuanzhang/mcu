#ifndef __MYQUEUE_H
#define __MYQUEUE_H

#include <stdint.h>
#include <string.h>

#define MYQUEUE_BUFF_MAX 4096
#define MYQUEUE_TYPE uint8_t


//静态队列.
typedef struct
{
    MYQUEUE_TYPE data[MYQUEUE_BUFF_MAX];     
    int head;    
    int tail; 
}MyQueue;

typedef enum
{
    myQueueFalse = 0,
    myQueueTrue
}MyQueueBool;


void myQueueInit(MyQueue* q);
	
MyQueueBool myQueueEmpty(MyQueue *q);

int myQueueNum(MyQueue *q);

MyQueueBool myQueueEnMember(MyQueue *q, MYQUEUE_TYPE x);

MyQueueBool myQueueEnMemberMult(MyQueue *q, MYQUEUE_TYPE *memberMult,int slen);

MyQueueBool myQueueDeMember(MyQueue *q, MYQUEUE_TYPE *x);

MyQueueBool myQueueDeMemberMult(MyQueue *q, MYQUEUE_TYPE *memberMult,int slen);

MyQueueBool myQueueMemberIndex(MyQueue *q, MYQUEUE_TYPE *member, int index);

MyQueueBool myQueueMemberMultIndex(MyQueue *q, MYQUEUE_TYPE *memberMult, int index, int slen);

MyQueueBool myQueueDeMemberIndex(MyQueue *q, MYQUEUE_TYPE *memberMult, int index);

MyQueueBool myQueueDeMemberMultIndex(MyQueue *q, MYQUEUE_TYPE *memberMult, int index, int slen );

int myQueueFindMember(MyQueue *q, MYQUEUE_TYPE x);

int myQueueFindMemberMult(MyQueue *q, MYQUEUE_TYPE *memberMult, int slen);

int myQueueDeFindMember(MyQueue *q, MYQUEUE_TYPE x);

int myQueueDeFindMemberMult(MyQueue *q, MYQUEUE_TYPE *memberMult, int slen);

#endif // !__MYQUEUE_H


