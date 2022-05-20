#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "myqueue.h"

// #include <stdlib.h>



void initQueue(QUEUE *queue_q)
{
    queue_q->BUF = (QUEUETYPE *)malloc(sizeof(QUEUETYPE)*BUF_SIZE);
    if(queue_q->BUF != NULL)     //队列内存分配成功
    {
        queue_q->front = queue_q->rear = 0; //初始化头尾指针
    }

}

//判空
bool isemptyQueue(QUEUE *queue_q)
{
    if(queue_q->front == queue_q->rear)
    {
        return true;
    }
    else
        return false;
}

//判满
bool is_fullQueue(QUEUE *queue_q)
{
    if((queue_q->rear +1)%BUF_SIZE == queue_q->front)
    {
        return true;
    }else
        return false;
}

//入队

void In_Queue(QUEUE *queue_q , QUEUETYPE value)
{
    if(is_fullQueue(queue_q) != true)        //队列未满
    {
        queue_q->BUF[queue_q->rear] = value;
        queue_q->rear = (queue_q->rear + 1)%BUF_SIZE ;    //尾指针偏移
    }
    else{
        queue_q->BUF[queue_q->rear] = value;
        queue_q->rear = (queue_q->rear + 1)%BUF_SIZE ;    //尾指针偏移
        queue_q->front = (queue_q->front + 1)%BUF_SIZE ;  //头指针指针偏移
    }
}


//出队
 void out_Queue(QUEUE *queue_q , QUEUETYPE *value)
 {
     if(isemptyQueue(queue_q) != true)        //队列未空
     {
        *value = queue_q->BUF[queue_q->front];
        queue_q->front = (queue_q->front + 1)%BUF_SIZE ;
     }else{
         *value = 0;
     }
}

void Queue_traversal(QUEUE *queue_q)
{
    if(isemptyQueue(queue_q) != true)
    {
        int ret=queue_q->front;
        while(ret != queue_q->rear)
        {
            // printf("%d  ",queue_q->BUF[ret]);
            ret=(ret+1)%BUF_SIZE;
        }
    }
}
