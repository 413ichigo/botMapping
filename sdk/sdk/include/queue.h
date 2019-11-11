#ifndef _QUEUE_H
#define _QUEUE_H

struct Queue* createQueue(unsigned capacity);

int isFull(struct Queue* queue);

int isEmpty(struct Queue* queue);

void enqueue(struct Queue* queue, int item);

int dequeue(struct Queue* queue);

int front(struct Queue* queue);

int rear(struct Queue* queue);

#endif
