#include "queue.h"
#include <string.h>
#include <stdlib.h>

void init(Queue* q) {
    q->head = q->trail = 0;
    //realloc(&(q->q), q->size * sizeof(q->q));
}

void push(Queue* q, char element[]) {
    strcpy(q->q[q->trail++ % q->size], element);
}

void pop(Queue* q, char* output) {
    strcpy(output, q->q[q->head++ % q->size]);
}

int empty(Queue* q) {
    return q->trail == q->head;
}

int full(Queue* q) {
    return (q->head - q->trail) % q->size + 1 == q->size;
}