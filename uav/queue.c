#include "queue.h"
#include <string.h>
#include <stdlib.h>

void queue_init(Queue* q, unsigned int message_size) {
    q->size = message_size;
    q->messages=malloc(message_size*sizeof(q->message));
    q->head = q->trail = 0;
    //realloc(&(q->q), q->size * sizeof(q->q));
}


void increase_message_size(char** messages, int size) {
	realloc(messages, 10 * sizeof(*messages));
}

void queue_push(Queue* q, char* element) {
	messages+(q->trail++ %q->size) = element
}

void queue_pop(Queue* q, char* output) {
	strcpy(output, *(messages + (q->head++ % q->size)));
}

int queue_empty(Queue* q) {
    return q->trail == q->head;
}

int queue_full(Queue* q) {
    return (q->head - q->trail) % q->size + 1 == q->size;
}