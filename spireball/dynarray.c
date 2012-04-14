#include <assert.h>
#include <string.h>
#include "dynarray.h"

void DYNA_initialize(DYNA_Array *array, size_t elementSize)
{
    assert(elementSize > 0);

    array->elementSize = elementSize;
    array->storage = 0;
    array->allocated = 0;
    array->count = 0;
}

void DYNA_deinitialize(DYNA_Array *array)
{
    free(array->storage);
}

void *DYNA_getStorage(DYNA_Array *array)
{
    return array->storage;
}

void enlarge(DYNA_Array *array)
{
    if (array->allocated)
    {
        array->allocated <<= 1;
    }
    else
    {
        array->allocated = 50;
    }
    array->storage = realloc(array->storage, array->allocated * array->elementSize);
}

void DYNA_add(DYNA_Array *array, void *element)
{
    if (array->count == array->allocated)
    {
        enlarge(array);
    }
    memcpy(
        array->storage + array->count * array->elementSize,
        element,
        array->elementSize
    );
    array->count++;
}

void DYNA_clear(DYNA_Array *array)
{
    array->count = 0;
}

int DYNA_getLength(const DYNA_Array *array)
{
    return array->count;
}


