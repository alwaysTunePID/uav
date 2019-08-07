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

void init(Queue* q);
void push(Queue* q, char element[]);
void pop(Queue* q, char* output);
int empty(Queue* q);
int full(Queue* q);

#endif