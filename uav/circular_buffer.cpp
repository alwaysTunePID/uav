/**
 * circular_buffer.cpp
 *
 * Used when collecting data, like sampling the IMU.
 * 
 * 
*/
#include "circular_buffer.h"
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>
#include "offset_memcpy.h"
#include "io_thread.h"


struct cbuffer_t {
    void *buffer;
    size_t write_pos;
    size_t read_pos;
    size_t max_elements; //of the buffer
    bool full;
    bool print; //enable logging
    size_t element_size; //size of each element in bytes
    pthread_mutex_t lock;
};



cbuffer_handle_t create_cbuffer()
{
    return (cbuffer_handle_t) new cbuffer_t[1];
}

int cbuffer_init(cbuffer_handle_t cbuf,size_t _max_elements,size_t _element_size)
{
    // Fill the data structure
    cbuf->buffer= ::operator new(_max_elements*_element_size);
    cbuffer_buf_reset(cbuf);
    cbuf->element_size=_element_size;
    cbuf->max_elements=_max_elements;
    cbuf->print = false;

    if (pthread_mutex_init(&cbuf->lock, NULL) != 0)
    {
        printio("Mutex init failed.");
        return -1;
    }

    return 0;

}

void cbuffer_buf_reset(cbuffer_handle_t cbuf)
{
    if(cbuf->print) printio("Reset buffer");
    cbuf->write_pos = 0;
    cbuf->read_pos = 0;
    cbuf->full = false;
}

void cbuffer_enable_print_outs(cbuffer_handle_t cbuf)
{
    cbuf->print = true;
    if(cbuf->print) printio("Print out enabled");
}

void cbuffer_disable_print_outs(cbuffer_handle_t cbuf)
{
    if(cbuf->print) printio("Print out disabled");
    cbuf->print = false;

}

int cbuffer_free(cbuffer_handle_t cbuf)
{
    if(cbuf->print) printio("Freeing buffer");
    ::operator delete(cbuf->buffer);
    pthread_mutex_destroy(&cbuf->lock);
    delete(cbuf);
    return 0;
}

size_t cbuffer_capacity(cbuffer_handle_t cbuf)
{
    return cbuf->max_elements;
}

bool cbuffer_full(cbuffer_handle_t cbuf)
{
    return cbuf->full;
}

bool cbuffer_empty(cbuffer_handle_t cbuf)
{
    return (!cbuf->full && (cbuf->write_pos == cbuf->read_pos));
}

size_t cbuffer_size(cbuffer_handle_t cbuf)
{

    size_t size = cbuf->max_elements;

    if(!cbuf->full)
    {
        if(cbuf->write_pos >= cbuf->read_pos)
        {
            size = (cbuf->write_pos - cbuf->read_pos);
        }
        else
        {
            size = (cbuf->max_elements + cbuf->write_pos - cbuf->read_pos);
        }
    }
    if(cbuf->print) printio("Query of size. Size is %u",size);
    return size;
}

// helper function for advancing write position and read position if buffer is full
static void advance_pointer(cbuffer_handle_t cbuf)
{
    if(cbuf->print) printio("Advancing pointer");

    if(cbuf->full)
    {
        cbuf->read_pos = (cbuf->read_pos + 1) % cbuf->max_elements;
    }

    cbuf->write_pos = (cbuf->write_pos + 1) % cbuf->max_elements;
    cbuf->full = (cbuf->write_pos == cbuf->read_pos);

    if (cbuf->full)
    {
        if(cbuf->print) printio("Buffer is now full");
    }
}

// helper function for advancing read position
static void retreat_pointer(cbuffer_handle_t cbuf)
{
    if(cbuf->print) printio("Retreating pointer");
    cbuf->full = false;
    cbuf->read_pos = (cbuf->read_pos + 1) % cbuf->max_elements;
}

int cbuffer_put(cbuffer_handle_t cbuf, void * in_data)
{
    pthread_mutex_lock(&cbuf->lock);
    if(cbuf->print) printio("Put-rutine got lock");

    memcpy_offset_dest(cbuf->buffer,in_data, cbuf->element_size * cbuf->write_pos, cbuf->element_size);
    if(cbuf->print) printio("Put data at pos %u",cbuf->write_pos );

    advance_pointer(cbuf);

    pthread_mutex_unlock(&cbuf->lock);
    if(cbuf->print) printio("Put-routine released lock");
    return 0;
}

int cbuffer_get(cbuffer_handle_t cbuf, void * out_data)
{

    if(!cbuffer_empty(cbuf))
    {
        pthread_mutex_lock(&cbuf->lock);
        if(cbuf->print) printio("Get-rutine got lock");

        memcpy_offset_source(out_data,cbuf->buffer, cbuf->element_size * cbuf->read_pos, cbuf->element_size);

        retreat_pointer(cbuf);

        pthread_mutex_unlock(&cbuf->lock);
        if(cbuf->print) printio("Get-routine released lock");
        return 0;
    }
    if(cbuf->print) printio("Attempted to read empty buffer");
    return -1; // should this be !?
}

int cbuffer_try_get(cbuffer_handle_t cbuf, void * out_data)
{
    if (cbuf == NULL) return -1;

    if(!cbuffer_empty(cbuf))
    {
        if(pthread_mutex_trylock(&cbuf->lock)) return -1; // buffer busy dont lock
        if(cbuf->print) printio("Get-rutine got lock");

        memcpy_offset_source(out_data,cbuf->buffer, cbuf->element_size * cbuf->read_pos, cbuf->element_size);

        retreat_pointer(cbuf);

        pthread_mutex_unlock(&cbuf->lock);
        if(cbuf->print) printio("Get-routine released lock");
        return 0;
    }
    if(cbuf->print) printio("Attempted to read empty buffer");
    return -1; // should this be !?
}



int cbuffer_top(cbuffer_handle_t cbuf, void * out_data)
{

    if(!cbuffer_empty(cbuf))
    {
        pthread_mutex_lock(&cbuf->lock);
        if(cbuf->print) printio("Top-rutine got lock");

        size_t temp_read_pos;
        if (cbuf->write_pos == 0){
            temp_read_pos = cbuf->max_elements -1;
        }
        else{
            temp_read_pos = cbuf->write_pos-1;
        }

        memcpy_offset_source(out_data,cbuf->buffer, cbuf->element_size * temp_read_pos, cbuf->element_size);


        pthread_mutex_unlock(&cbuf->lock);

        if(cbuf->print) printio("Top-routine released lock");
        return 0;
    }
    if(cbuf->print) printio("Attempted to read empty buffer");
    return -1; // should this be !?
}









