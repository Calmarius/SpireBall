#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>

/**
 * Represents an AVL tree.
 */
typedef struct AVL_Node
{
    struct AVL_Node *parent;
    struct AVL_Node *left;
    struct AVL_Node *right;
    int leftHeight;
    int rightHeight;
    void *key;
    void *value;
} AVL_Node;

typedef int (*AVL_Comparer)(void *a, void *b);

typedef struct
{
    AVL_Node *rootNode;
    AVL_Comparer keyComparer;
} AVL_Tree;

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

NodeAddResult addToNode(AVL_Tree *tree, AVL_Node *node, void *key, void *value)
{
    int tmp = tree->keyComparer(key, node->key);
    AVL_Node **nodeToAddTo;
    int ballanceMod;
    NodeAddResult retVal;

    if (tmp < 0)
    {
        nodeToAddTo = &node->left;
        ballanceMod = 1;
    }
    else if (tmp > 0)
    {
        nodeToAddTo = &node->right;
        ballanceMod = -1;
    }
    else
    {
        return EXISTS;
    }
    if (*nodeToAddTo)
    {
        retVal = addToNode(tree, *nodeToAddTo, key, value);
        if (ballanceMod > 0)
        {
            node->leftHeight =
                node->left ?
                    (
                        node->left->leftHeight > node->left->rightHeight ?
                        node->left->leftHeight :
                        node->left->rightHeight
                    ) + 1: 0;
        }
        else
        {
            node->rightHeight =
                node->right ?
                    (
                        node->right->leftHeight > node->right->rightHeight ?
                        node->right->leftHeight :
                        node->right->rightHeight
                    ) + 1: 0;

        }
    }
    else
    {
        retVal = ADDED;
        if (!node->left && !node->right)
        {
            if (ballanceMod > 0)
            {
                node->leftHeight++;
            }
            else
            {
                node->rightHeight++;
            }
            retVal = ADDED_HEIGHT_INCREASED;
        }
        *nodeToAddTo = allocateNode();
        (*nodeToAddTo)->parent = node;
        (*nodeToAddTo)->key = key;
        (*nodeToAddTo)->value = value;
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
    return retVal;
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

void AVL_delete(AVL_Tree *tree, void *key)
{

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

