#include "queue.h"
#include <string.h>
#include <stdlib.h>

void queue_init(Queue* q) {
    q->head = q->trail = 0;
    //realloc(&(q->q), q->size * sizeof(q->q));
}

void queue_push(Queue* q, char element[]) {
    strcpy(q->q[q->trail++ % q->size], element);
}

void queue_pop(Queue* q, char* output) {
    strcpy(output, q->q[q->head++ % q->size]);
}

int queue_empty(Queue* q) {
    return q->trail == q->head;
}

int queue_full(Queue* q) {
    return (q->head - q->trail) % q->size + 1 == q->size;
}