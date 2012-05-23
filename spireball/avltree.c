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
    const void *key;
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

/**
 * Rotates the node left.
 *
 * @param [in,out] tree The AVL tree.
 * @param [in,out] node The node to rotate.
 */
static void rotateLeft(AVL_Tree *tree, AVL_Node *node)
{
    /*
    Intial node is like this:
         A
        /  \
      B     C
     / \   / \
    D   E F   G

    Turned into this:

          C
         / \
        A   G
       / \
      B   F
     / \
    D   E
    */
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

/**
 * Rotates the node right.
 *
 * @param [in,out] tree The AVL tree.
 * @param [in,out] node The node to rotate.
 */
static void rotateRight(AVL_Tree *tree, AVL_Node *node)
{
    /*
    Intial node is like this:
         A
        /  \
      B     C
     / \   / \
    D   E F   G

    Turned into this:

      B
     / \
    D   A
       / \
      E   C
         / \
        F   G
    */
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

/**
 * Dumps a single node.
 *
 * @param [in] node The node to dump.
 * @param [in] depth The depth of the node.
 * @param [in] dir which subtree (negative: left, positive: right)
 * @param [in] dumpFn Dump function first argument is the key, second is the value.
 */
static void dumpNode(const AVL_Node *node, int depth, char dir, void (*dumpFn)(const void*,const void*))
{
    if (!node) return;
    dumpFn(node->key, node->value);
}

/**
 * Dumps an entire subtree
 *
 * @param [in] node The root node of the subtree.
 * @param [in] depth The depth of the node.
 * @param [in] dir which subtree (negative: left, positive: right)
 * @param [in] dumpFn Dump function first argument is the key, second is the value.
 */
static void dumpSubtree(const AVL_Node *node, int depth, char dir, void (*dumpFn)(const void*, const void*))
{
    if (node) dumpSubtree(node->left, depth + 1, -1, dumpFn);
    dumpNode(node, depth, dir, dumpFn);
    if (node) dumpSubtree(node->right, depth + 1, 1, dumpFn);
}

void AVL_dumpTree(const AVL_Tree *tree, void (*dumpFn)(const void*, const void*))
{
    dumpSubtree(tree->rootNode, 0, 0, dumpFn);
    printf("\n");
}

/**
 * Reballances the tree, starting from the given node.
 *
 * @param [in,out] tree The AVL tree to reballance.
 * @param [in,out] node The node to start reballancing.
 *
 * @note This function is recursively called on the parent node of the given node.
 */
static void reballance(AVL_Tree *tree, AVL_Node *node)
{
    if (!node) return ;
    // Calculate the height of the subtrees.
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
    // Reballance the node when needed.
    {
        int ballanceFactor = node->leftHeight - node->rightHeight;
        if (ballanceFactor < -1)
        {
            int subBallanceFactor =
                node->right->leftHeight - node->right->rightHeight;
            // right subtree is higher
            if (subBallanceFactor == 1)
            {
                /*
                    Right-left case.
                    B is the current node
                      B
                     / \
                    D   A
                       / \
                      E   C
                     / \
                    F   G
                */
                // Rotate A right to get a right-right case.
                rotateRight(tree, node->right);
            }
            /*
                Right-right case
                  B
                 / \
                D   E
                   / \
                  F   A
                     / \
                    G   C
            */
            // Rotate B left to reballance the node.
            rotateLeft(tree, node);
        }
        else if (ballanceFactor > 1)
        {
            int subBallanceFactor =
                node->left->leftHeight - node->left->rightHeight;
            // right subtree is higher
            if (subBallanceFactor == -1)
            {
                /*
                    Left-right case
                      C
                     / \
                    A   G
                   / \
                  B   F
                     / \
                    D   E
                */
                // Rotate A left to get a left-left case.
                rotateLeft(tree, node->left);
            }
            /*
                Left-left case
                      C
                     / \
                    F   G
                   / \
                  A   E
                 / \
                B   D
            */
            // Rotate C right to reballance the node.
            rotateRight(tree, node);
        }
    }
    // Do the same with the parent node.
    reballance(tree, node->parent);
}

/**
 * Adds new element to the subtree.
 *
 * @param [in,out] tree AVL tree.
 * @param [in,out] node Root of the subtree to add to.
 * @param [in] key The key of the element.
 * @param [in,out] value The value of the element.
 *
 * @retval ADDED The node succesfully added.
 * @retval EXISTS The node is already exists in the AVL tree.
 */
static NodeAddResult addToNode(AVL_Tree *tree, AVL_Node *node, const void *key, void *value)
{
    int tmp = tree->keyComparer(key, node->key);
    AVL_Node **nodeToAddTo = 0;

    // Find the key in the tree recursively, and add it to a leaf node.
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
    // Reballance the leaf node.
    reballance(tree, *nodeToAddTo);

    return ADDED;
}

void AVL_add(AVL_Tree *tree, const void *key, void *value)
{
    if (!tree->rootNode)
    {
        // First element.
        tree->rootNode = allocateNode();
        tree->rootNode->key = key;
        tree->rootNode->value = value;
    }
    else
    {
        // Not the first element.
        addToNode(tree, tree->rootNode, key, value);
    }
}

/**
 * Performs lookup on the subtree of the AVL tree.
 *
 * @param [in] tree the AVL tree.
 * @param [in] node The root of the subtree.
 * @param [in] key The key to find.
 *
 * @note This function is recursively called on the subtrees.
 *
 * @return Pointer to the node with the given key, NULL if it's not found in the subtree.
 */
static const AVL_Node *lookup(const AVL_Tree *tree, const AVL_Node *node, const void *key)
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

const AVL_Node *AVL_find(const AVL_Tree *tree, const void *key)
{
    return lookup(tree, tree->rootNode, key);
}

/**
 * Removes node from the tree
 *
 * @param [in,out] tree The AVL tree.
 * @param [in,out] node The node to remove.
 *
 * TODO: Continue documentation here!
 */
static void removeNode(AVL_Tree *tree, AVL_Node *node)
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

void deleteFrom(AVL_Tree *tree, AVL_Node *node, const void *key)
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

void AVL_delete(AVL_Tree *tree, const void *key)
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

const AVL_Node *AVL_getLeast(const AVL_Tree *tree)
{
    AVL_Node *current = tree->rootNode;
    if (!current) return 0;
    while (current->left)
    {
        current = current->left;
    }
    return current;
}

const AVL_Node *AVL_getGreatest(const AVL_Tree *tree)
{
    AVL_Node *current = tree->rootNode;
    if (!current) return 0;
    while (current->right)
    {
        current = current->right;
    }
    return current;
}

const void *AVL_getKey(const AVL_Node *node)
{
    return node->key;
}

void *AVL_getValue(const AVL_Node *node)
{
    return node->value;
}

