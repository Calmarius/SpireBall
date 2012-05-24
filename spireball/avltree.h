/**
 * @file avltree.h
 * AVL tree module. Provides data structure for quick insert, find and removal.
 */

#ifndef AVLTREE_H
#define AVLTREE_H

struct AVL_Node;
typedef struct AVL_Node AVL_Node;

/**
 * Comparer function type.
 *
 * @param [in] a,b The two element to compare.
 *
 * @retval negative on a < b.
 * @retval 0 on a == b
 * @retval positive on a > b
 */
typedef int (*AVL_Comparer)(const void *a, const void *b);

/**
 * Struct representing an AVL tree.
 */
typedef struct
{
    AVL_Node *rootNode; ///< Pointer to the root node
    AVL_Comparer keyComparer; ///< Key comparer function
} AVL_Tree;

/**
 * Initializes an AVL tree
 *
 * @param [in,out] tree The AVL tree to initialize.
 * @param [in] comparer The comparer function.
 */
void AVL_initialize(AVL_Tree *tree, AVL_Comparer comparer);
/**
 * Dumps the AVL tree.
 *
 * @param [in] tree The AVL tree.
 * @param [in] dumpFn Dump function, the first argument is the pointer to the
 *      key, the second is the pointer to the value.
 *
 */
void AVL_dumpTree(const AVL_Tree *tree, void (*dumpFn)(const void*,const void*));

/**
 * Adds a new element to the AVL tree.
 *
 * @param [in,out] tree The AVL tree.
 * @param [in] key, value Pointer to the key and the value.
 *
 * @note The key pointer must be valid during the AVL tree's lifetime and it's
 *      value must be unchanged.
 */
void AVL_add(AVL_Tree *tree, const void *key, void *value);

/**
 * Finds an element in the AVL tree by key.
 *
 * @param [in] tree The AVL tree to find in.
 * @param [in] key The key to find.
 *
 * @return Pointer to the node if the element is found or NULL if not found.
 */
const AVL_Node *AVL_find(const AVL_Tree *tree, const void *key);

/**
 * Deletes an element from the AVL tree.
 *
 * @param [in,out] tree The AVL tree.
 * @param [in] key The key of the element to delete.
 */
void AVL_delete(AVL_Tree *tree, const void *key);

/**
 * Deinitializes the AVL tree, and releases the associated resources.
 * The pointer to the tree will be invalid.
 *
 * @param [in,out] tree The AVL tree to delete.
 */
void AVL_deinitialize(AVL_Tree *tree);

/**
 * Gets the element from the tree with the lowest key.
 *
 * @param [in] tree The AVL tree.
 *
 * @return The node of the lowest key element, or NULL if the tree is empty.
 */
const AVL_Node *AVL_getLeast(const AVL_Tree *tree);

/**
 * Gets the element from the tree with the greatest key.
 *
 * @param [in] tree The AVL tree.
 *
 * @return The node with the greatest key value, or NULL if the tree is empty.
 */
const AVL_Node *AVL_getGreatest(const AVL_Tree *tree);

/**
 * @param [in] node A node pointer we got from the AVL_getLeast or AVL_getGreatest
 *
 * @return The pointer to the key from it.
 */
const void *AVL_getKey(const AVL_Node *node);

/**
 * @param [in] node A node pointer we got from the AVL_getLeast or AVL_getGreatest
 *
 * @return The pointer to the value in it.
 */
void *AVL_getValue(const AVL_Node *node);

/**
 * Deletes all elements from the AVL tree.
 *
 * @param [in,out] tree The AVL tree.
 */
void AVL_clear(AVL_Tree *tree);

#endif // AVLTREE_H
