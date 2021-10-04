#pragma once

#include <stdbool.h>
#define BUF_SIZE 4

typedef float QUEUETYPE;

typedef struct Queue
{
    QUEUETYPE * BUF;
    int front;
    int rear;
}QUEUE;


void initQueue(QUEUE *queue_q,uint8_t bufsize);

//判空
bool isemptyQueue(QUEUE *queue_q);

//判满
bool is_fullQueue(QUEUE *queue_q);

//入队

void In_Queue(QUEUE *queue_q , QUEUETYPE value);


//出队
void out_Queue(QUEUE *queue_q , QUEUETYPE *value);

void Queue_traversal(QUEUE *queue_q);
//读取队列中的一个值
QUEUETYPE read_Queue(QUEUE *queue_q , uint8_t ID);
