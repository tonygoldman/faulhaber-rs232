#ifndef MC_HELPERS_H
#define MC_HELPERS_H

/*-----------------------------------------
 * define a type for a function pointer either having
 * no parameter in the call or having a pointer object as an
 * argument which can be used to hand over a received message
 * 
 * -------------------------------------------------------*/

#include <stdint.h>

//define a void function pointer which takes an argument
typedef void* (*ifunction_pointer_t)(void *op, int index);

struct ifunction_holder {
    ifunction_pointer_t callback;
    void *op;
};

typedef void* (*vfunction_pointer_t)(void *op);

struct vfunction_holder {
    vfunction_pointer_t callback;
    void *op;
};

typedef void* (*pfunction_pointer_t)(void *op, void *p);

struct pfunction_holder {
    pfunction_pointer_t callback;
    void *op;
};

//a function piter suitable to had over the information of a topic
typedef void* (*tfunction_pointer_t)(void *op, void *t, uint32_t l);

struct tfunction_holder {
    tfunction_pointer_t callback;
    void *op;
};

#endif
