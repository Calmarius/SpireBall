#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

#include "avltree.h"

/**
 * Represents an AVL tree.
 */
struct AVL_Node
{
    struct AVL_Node *parent;
    struct AVL_Node *left;
    struct AVL_Node *right;
    int leftHeight;
    int rightHeight;
    void *key;
    void *value;
};

void AVL_initialize(AVL_Tree *tree, AVL_Comparer comparer)
{
    tree->rootNode = 0;
    tree->keyComparer = comparer;
}

AVL_Node *allocateNode()
{
    AVL_Node *node = malloc(sizeof(AVL_Node));
    node->parent = 0;
    node->left = 0;
    node->right = 0;
    node->leftHeight = 0;
    node->rightHeight = 0;
    return node;
}

typedef enum
{
    EXISTS,
    ADDED,
    ADDED_HEIGHT_INCREASED
} NodeAddResult;

void rotateLeft(AVL_Tree *tree, AVL_Node *node)
{
    AVL_Node *pivot = node->right;
    AVL_Node *sub;
    assert(pivot);

    if (node->parent)
    {
        if (node->parent->left == node) node->parent->left = pivot;
        if (node->parent->right == node) node->parent->right = pivot;
    }

    sub = pivot->left;
    pivot->parent = node->parent;
    pivot->left = node;
    node->right = sub;
    node->parent = pivot;
    if (sub) sub->parent = node;

    node->rightHeight = pivot->leftHeight;
    pivot->leftHeight = node->leftHeight + 1;

    if (tree->rootNode == node) tree->rootNode = pivot;
}

void rotateRight(AVL_Tree *tree, AVL_Node *node)
{
    AVL_Node *pivot = node->left;
    AVL_Node *sub;
    assert(pivot);

    if (node->parent)
    {
        if (node->parent->left == node) node->parent->left = pivot;
        if (node->parent->right == node) node->parent->right = pivot;
    }

    sub = pivot->right;
    pivot->parent = node->parent;
    pivot->right = node;
    node->left = sub;
    node->parent = pivot;
    if (sub) sub->parent = node;

    node->leftHeight = pivot->rightHeight;
    pivot->rightHeight = node->rightHeight + 1;

    if (tree->rootNode == node) tree->rootNode = pivot;
}

void dumpNode(AVL_Node *node, int depth, char dir)
{
    if (!node)
    {
        printf("%*s %s (null)\n", depth, " ", dir == -1? "left: " : ( dir == 1 ? "right: " : ""));
        return;
    }
    printf(
        "%*s %s %d %s [%d %d]\n",
        depth,
        " ",
        dir == -1? "left: " : ( dir == 1 ? "right: " : ""),
        *(int*)node->key,
         (char*)node->value,
         node->leftHeight,
         node->rightHeight
    );
}

void dumpSubtree(AVL_Node *node, int depth, char dir)
{
    dumpNode(node, depth, dir);
    if (node) dumpSubtree(node->left, depth + 3, -1);
    if (node) dumpSubtree(node->right, depth + 3, 1);
}

void AVL_dumpTree(AVL_Tree *tree)
{
    dumpSubtree(tree->rootNode, 0, 0);
    printf("\n");
}

void reballance(AVL_Tree *tree, AVL_Node *node)
{
    if (!node) return ;
    if (node->left)
    {
        node->leftHeight =
            (node->left->leftHeight > node->left->rightHeight ?
            node->left->leftHeight : node->left->rightHeight) + 1 ;
    }
    else
    {
        node->leftHeight = 0;
    }
    if (node->right)
    {
        node->rightHeight =
            (node->right->leftHeight > node->right->rightHeight ?
            node->right->leftHeight : node->right->rightHeight) + 1;
    }
    else
    {
        node->rightHeight = 0;
    }
    {
        int ballanceFactor = node->leftHeight - node->rightHeight;
        if (ballanceFactor < -1)
        {
            int subBallanceFactor =
                node->right->leftHeight - node->right->rightHeight;
            // right subtree is higher
            if (subBallanceFactor == 1)
            {
                // right-left case extra rotation needed.
                rotateRight(tree, node->right);
            }
            rotateLeft(tree, node);
        }
        else if (ballanceFactor > 1)
        {
            int subBallanceFactor =
                node->left->leftHeight - node->left->rightHeight;
            // right subtree is higher
            if (subBallanceFactor == -1)
            {
                // left-right case extra rotation needed.
                rotateLeft(tree, node->left);
            }
            rotateRight(tree, node);
        }
    }
    reballance(tree, node->parent);
}

NodeAddResult addToNode(AVL_Tree *tree, AVL_Node *node, void *key, void *value)
{
    int tmp = tree->keyComparer(key, node->key);
    AVL_Node **nodeToAddTo = 0;

    if (tmp < 0)
    {
        if (node->left)
        {
            return addToNode(tree, node->left, key, value);
        }
        else
        {
            nodeToAddTo = &node->left;
        }
    }
    else if (tmp > 0)
    {
        if (node->right)
        {
           return addToNode(tree, node->right, key, value);
        }
        else
        {
            nodeToAddTo = &node->right;
        }
    }
    else
    {
        return EXISTS;
    }
    *nodeToAddTo = allocateNode();
    (*nodeToAddTo)->parent = node;
    (*nodeToAddTo)->key = key;
    (*nodeToAddTo)->value = value;
    reballance(tree, *nodeToAddTo);

    return ADDED;
}

void AVL_add(AVL_Tree *tree, void *key, void *value)
{
    if (!tree->rootNode)
    {
        tree->rootNode = allocateNode();
        tree->rootNode->key = key;
        tree->rootNode->value = value;
    }
    else
    {
        addToNode(tree, tree->rootNode, key, value);
    }
}

AVL_Node *lookup(AVL_Tree *tree, AVL_Node *node, void *key)
{
    int tmp;

    if (!node)
    {
        return 0;
    }

    tmp = tree->keyComparer(key, node->key);
    if (tmp < 0)
    {
        return lookup(tree, node->left, key);
    }
    else if (tmp > 0)
    {
        return lookup(tree, node->right, key);
    }
    return node;
}

AVL_Node *AVL_find(AVL_Tree *tree, void *key)
{
    return lookup(tree, tree->rootNode, key);
}

void removeNode(AVL_Tree *tree, AVL_Node *node)
{
    // Move the inorder successor or predecessor to
    // the position of the deleted element.
    AVL_Node *nodeToMove = 0;
    AVL_Node *nodeToUpdate = 0;
    if (node->left)
    {
        nodeToMove = node->left;
        while (nodeToMove->right)
        {
            nodeToMove = nodeToMove->right;
        }
    }
    else if (node->right)
    {
        nodeToMove = node->right;
        while (nodeToMove->left)
        {
            nodeToMove = nodeToMove->left;
        }
    }
    if (nodeToMove)
    {
        // Remove the node to move from the tree.
        nodeToUpdate = nodeToMove->parent;
        removeNode(tree, nodeToMove);
        // update links
        nodeToMove->parent = node->parent;
        nodeToMove->left = node->left;
        nodeToMove->right = node->right;
        if (nodeToMove->left) nodeToMove->left->parent = nodeToMove;
        if (nodeToMove->right) nodeToMove->right->parent = nodeToMove;
        if (nodeToMove->parent)
        {
            if (nodeToMove->parent->left == node) nodeToMove->parent->left = nodeToMove;
            if (nodeToMove->parent->right == node) nodeToMove->parent->right = nodeToMove;
        }
        if (nodeToUpdate != node)
        {
            reballance(tree, nodeToUpdate); //< continue here.
        }
        else
        {
            reballance(tree, nodeToMove);
        }
    }
    else
    {
        if (node->parent)
        {
            if (node->parent->left == node) node->parent->left = 0;
            if (node->parent->right == node) node->parent->right = 0;
            reballance(tree, node->parent);
        }
    }
    if (tree->rootNode == node)
    {
        tree->rootNode = nodeToMove;
    }
}

void deleteFrom(AVL_Tree *tree, AVL_Node *node, void *key)
{
    int result;

    if (!node) return;
    result = tree->keyComparer(key, node->key);

    if (result < 0)
    {
        return deleteFrom(tree, node->left, key); // void functions can return void, obviously.
    }
    else if (result > 0)
    {
        return deleteFrom(tree, node->right, key);
    }
    // Found
    removeNode(tree, node);
    free(node);
}

void AVL_delete(AVL_Tree *tree, void *key)
{
    deleteFrom(tree, tree->rootNode, key);
}

void destroy(AVL_Node *node)
{
    if (!node) return;
    destroy(node->left);
    destroy(node->right);
    free(node);
}

void AVL_deinitialize(AVL_Tree *tree)
{
    destroy(tree->rootNode);
}

void AVL_clear(AVL_Tree *tree)
{
    destroy(tree->rootNode);
    tree->rootNode = 0;
}

AVL_Node *AVL_getLeast(AVL_Tree *tree)
{
    AVL_Node *current = tree->rootNode;
    if (!current) return 0;
    while (current->left)
    {
        current = current->left;
    }
    return current;
}

AVL_Node *AVL_getGreatest(AVL_Tree *tree)
{
    AVL_Node *current = tree->rootNode;
    if (!current) return 0;
    while (current->right)
    {
        current = current->right;
    }
    return current;
}

void *AVL_getKey(AVL_Node *node)
{
    return node->key;
}

void *AVL_getValue(AVL_Node *node)
{
    return node->value;
}

