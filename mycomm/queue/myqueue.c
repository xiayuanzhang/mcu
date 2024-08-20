#include "myqueue.h"

//队列中, tail+1==head,队列满,禁止入栈
//head==tail,队列空

//方便计算	
#define MYQUEUE_ADD(exp1,exp2) (((exp1) + (exp2)) % MYQUEUE_BUFF_MAX)
#define MYQUEUE_SUB(exp1,exp2) (((exp1) - (exp2) + MYQUEUE_BUFF_MAX) % MYQUEUE_BUFF_MAX)
//减少函数调用,提高效率
#define MYQUEUE_FULL(q) MYQUEUE_ADD(q->tail,1) == q->head
#define MYQUEUE_EMPTY(q) q->head == q->tail



void myQueueInit(MyQueue* q)
{
	q->head = 0;
	q->tail = 0;
	memset(q->data,0,sizeof(q->data));
	
}

//判断丢列是否为空
MyQueueBool myQueueEmpty(MyQueue* q)
{
	if (MYQUEUE_EMPTY(q))	//队空条件
		return myQueueTrue;
	else
		return myQueueFalse;
}

MyQueueBool myQueueFull(MyQueue* q)
{
	if(MYQUEUE_FULL(q))	//队满条件
		return myQueueTrue;
	else
		return myQueueFalse;
}


// 队列中元素的个数
int myQueueNum(MyQueue* q)
{
	return MYQUEUE_SUB(q->tail,q->head);
}


// 入队
MyQueueBool myQueueEnMember(MyQueue* q, MYQUEUE_TYPE x)
{
	//队列满则报错
	if(MYQUEUE_FULL(q))
		return myQueueFalse;
	//将x插入队尾	
	q->data[q->tail] = x;		
	q->tail = MYQUEUE_ADD(q->tail,1);
	return myQueueTrue;
}

// 入队多个成员
MyQueueBool myQueueEnMemberMult(MyQueue* q, MYQUEUE_TYPE *memberMult,int slen)
{
	for(int i = 0;i < slen;i++)
	{
		if(MYQUEUE_FULL(q))
			return myQueueFalse;
		//将x插入队尾	
		q->data[q->tail] = memberMult[i];		
		q->tail = MYQUEUE_ADD(q->tail,1);
	}
	return myQueueTrue;
}

// 出队
MyQueueBool myQueueDeMember(MyQueue* q, MYQUEUE_TYPE *x)
{
	if (MYQUEUE_EMPTY(q))
		return myQueueFalse;	//队空则报错
	*x = q->data[q->head];
	q->head =MYQUEUE_ADD(q->head,1);
	return myQueueTrue;
}

// 出队
MyQueueBool myQueueDeMemberMult(MyQueue* q, MYQUEUE_TYPE *memberMult,int slen)
{
	for(int i = 0;i < slen;i++)
	{
		if (MYQUEUE_EMPTY(q))
			return myQueueFalse;	//队空则报错
		memberMult[i] = q->data[q->head];
		q->head =MYQUEUE_ADD(q->head,1);
		
	}
	return myQueueTrue;
}


/**
 * @brief 获取index位置的单个成员,不出队(队头不变)
 * 
 * @param q 
 * @param member 
 * @param index 从0开始索引, 并非相对对头的index,而是相对队列数组[0] 的index. 如果index>队列长度,自动循环计算
 * @return MyQueueBool 
 */
MyQueueBool myQueueMemberIndex(MyQueue* q, MYQUEUE_TYPE *member,int index)
{
	index = MYQUEUE_ADD(0,index); //index相对于队列数组[0] 的index, 并非相对对头的index

	if (MYQUEUE_EMPTY(q))
		return myQueueFalse;	//队空则报错
	*member = q->data[index];
	return myQueueTrue;
}

/**
 * @brief 获取index位置的多个成员,不出队(队头不变)
 * 
 * @param q 
 * @param member 
 * @param index 从0开始索引, 并非相对head的index,而是相对队列数组[0] 的index. 如果index>队列长度,自动循环计算
 * @return MyQueueBool 
 */
MyQueueBool myQueueMemberMultIndex(MyQueue* q, MYQUEUE_TYPE *memberMult,int index,int slen)
{
	index = MYQUEUE_ADD(0,index); //index相对于队列数组[0] 的index, 并非相对对头的index
	for(int i = 0;i < slen;i++)
	{
		if (MYQUEUE_EMPTY(q))
			return myQueueFalse;	//队空则报错
		index = MYQUEUE_ADD(index,1);
		memberMult[i] = q->data[index];
	}
	return myQueueTrue;
}

//从index位置出队单成员,index之前的数据全部丢弃

/**
 * @brief 从index位置出队单成员,index之前的数据全部丢弃
 * 
 * @param q 
 * @param memberMult 
 * @param index 从0开始索引, 并非相对head的index,而是相对队列数组[0] 的index
 * @return MyQueueBool 
 */
MyQueueBool myQueueDeMemberIndex(MyQueue* q, MYQUEUE_TYPE *memberMult,int index)
{
	q->head = index;
	return myQueueDeMember(q,memberMult);
}

//从index位置出队多成员,index之前的数据全部丢弃
MyQueueBool myQueueDeMemberMultIndex(MyQueue* q, MYQUEUE_TYPE *memberMult,int index,int slen)
{
	q->head = index;
	return myQueueDeMemberMult(q,memberMult,slen);
}

/**
 * @brief 查找字符在队列中的位置,但不出队
 * 
 * @param q 
 * @param x 
 * @return int 返回值是相对于队列数组[0] 的index, 并非相对head的index
 */
int myQueueFindMember(MyQueue* q, MYQUEUE_TYPE x)
{
	int index= q->head;
	int num = myQueueNum(q);
	while(num--)
	{
		if(q->data[index] == x)
			return index;
		index  = MYQUEUE_ADD(index , 1);
	}
	return -1;
}





//查找字符串在队列中的位置
//返回第一个字符所在位置
int myQueueFindMemberMult(MyQueue* q, MYQUEUE_TYPE* memberMult,int slen)
{
	int count = 0;
	int index= q->head;
	int num = myQueueNum(q);
	while(num--)
	{
		if(q->data[index] == memberMult[count])
		{
			count++;
		}
		else
		{
			count = 0;	
		}
		
		if(count == slen)
		{
			count--; //从数量变道索引值
			return MYQUEUE_SUB(index , count);
		}
		//不管是否相等,index都向后偏移
		index  = MYQUEUE_ADD(index , 1);
	}
	return -1;
}


//查找字符在队列中的位置
//丢弃查找过的数据
int myQueueDeFindMember(MyQueue* q, MYQUEUE_TYPE x)
{
	int num = myQueueNum(q);
	while(num--)
	{
		if(x == q->data[q->head])
			return q->head;
		q->head = MYQUEUE_ADD(q->head,1);
	}
	return -1;
}

/**
 * @brief 查找 memberMult 在队列中的位置
 * 丢弃查找过的非 memberMult 数据. 
 * 如果memberMult长度为3, 那么丢弃memberMult[0]之前的数据, 返回memberMult[0]所在队列中的index
 * 
 * @param q 待查找队列
 * @param memberMult 待查找的成员的指针 
 * @param slen 待查找的成员的长度
 * @return int -1:没有找到, 其他:memberMult[0]所在index
 */
int myQueueDeFindMemberMult(MyQueue* q, MYQUEUE_TYPE* memberMult,int slen)
{
	//使用index暂存计数
	//当取出内容和要查找的对不上的时候再偏移head,否则不偏移
	int count = 0;
	int index = q->head;
	int num = myQueueNum(q);
	while(num--)
	{
		if(q->data[index] == memberMult[count])
		{
			count++; //count = 有多少个相同
		}
		else//第count+1个不相等
		{
			//不相等,丢弃数据
			q->head = MYQUEUE_ADD(index , 1); //当前index不相等,所以head=index向后偏移1
			count = 0;
		}
		if(count == slen)
		{
			count--; //从数量变道索引值
			return MYQUEUE_SUB(index , count);
		}
		
		//不管是否相等,index都向后偏移
		index  = MYQUEUE_ADD(index , 1); 
	}
	return -1;
}
