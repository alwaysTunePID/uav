#ifndef QUEUE_H
#define QUEUE_H

typedef union Data {
    int d;
    double lf;
    char str[255];
} Data;

typedef struct Queue {
    unsigned int size;
    unsigned int head;
    unsigned int trail;
    char q[10][255];
    void* overfull;
} Queue;

void queue_init(Queue* q);
void queue_push(Queue* q, char element[]);
void queue_pop(Queue* q, char* output);
int queue_empty(Queue* q);
int queue_full(Queue* q);

#endif