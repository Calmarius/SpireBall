#ifndef AVLTREE_H
#define AVLTREE_H

struct AVL_Node;
typedef struct AVL_Node AVL_Node;

typedef int (*AVL_Comparer)(void *a, void *b);
typedef struct
{
    AVL_Node *rootNode;
    AVL_Comparer keyComparer;
} AVL_Tree;

void AVL_initialize(AVL_Tree *tree, AVL_Comparer comparer);

void AVL_dumpTree(AVL_Tree *tree);

void AVL_add(AVL_Tree *tree, void *key, void *value);

AVL_Node *AVL_find(AVL_Tree *tree, void *key);

void AVL_delete(AVL_Tree *tree, void *key);

void AVL_deinitialize(AVL_Tree *tree);

AVL_Node *AVL_getLeast(AVL_Tree *tree);

AVL_Node *AVL_getGreatest(AVL_Tree *tree);

void *AVL_getKey(AVL_Node *node);

void *AVL_getValue(AVL_Node *node);

void AVL_clear(AVL_Tree *tree);

#endif // AVLTREE_H
