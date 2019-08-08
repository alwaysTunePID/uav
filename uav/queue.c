#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void queue_init(Queue* q, unsigned int message_size) {
    q->size = message_size;
    q->length = 255;
    q->messages=malloc(message_size*q->length*sizeof(char));
    q->head = q->trail = 0;
}

void queue_push(Queue* q, char* element) {
    char* mem = malloc(q->length*sizeof(char));
    strcpy(mem, element);
	if(!queue_full(q)) *(q->messages + (q->trail++ %q->size)) = mem;
}

void queue_pop(Queue* q, char* output) {
	if(!queue_empty(q)) strcpy(output, *(q->messages + (q->head++ % q->size)));
    free(*(q->messages + (q->head - 1 % q->size)));
}

int queue_empty(Queue* q) {
    return q->trail == q->head;
}

int queue_full(Queue* q) {
    return (q->head - q->trail) % q->size + 1 == q->size;
}