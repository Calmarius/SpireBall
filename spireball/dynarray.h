#ifndef DYNARRAY_H
#define DYNARRAY_H

#include <stdlib.h>

/**
 * Represents a dynamic array.
 */
typedef struct
{
    char *storage;
    int count;
    int allocated;
    size_t elementSize;
} DYNA_Array;

/**
 * Initializes a dynamic array
 *
 * @param [in,out] array The array.
 * @param [in] elementSize Size of an element.
 */
void DYNA_initialize(DYNA_Array *array, size_t elementSize);

/**
 * Deinitializes the dynamic array.
 *
 * @param [in,out] array The array.
 */
void DYNA_deinitialize(DYNA_Array *array);

/**
 * Gets the storage area of the array, casted to the appropriate pointer,
 * the contents of the array can be accessed.
 *
 * @param [in,out] array The array.
 *
 * @return A pointer to the storage.
 */
void *DYNA_getStorage(DYNA_Array *array);

/**
 * Adds an element to the array.
 *
 * @param [in,out] array The array.
 * @param [in] from pointer to the element to add.
 */
void DYNA_add(DYNA_Array *array, void *element);

/**
 * Clears an element from the array.
 *
 * @param [in,out] array The array.
 */
void DYNA_clear(DYNA_Array *array);

/**
 * Gets the count of elements of the array.
 *
 * @param [in] array The array
 *
 * @return Count of elements.
 */
int DYNA_getLength(const DYNA_Array *array);

#endif // DYNARRAY_H
