#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include "MinHeapNode.h"

// A class for Min Heap
class MinHeap
{
    MinHeapNode **heap; // pointer to array of elements in heap
    int capacity;       // maximum possible size of min heap
    int size;           // Current number of elements in min heap

public:
    MinHeap(int capacity);

    ~MinHeap();

    // to heapify a subtree with the root at given index
    void MinHeapify(int);

    int parent(int i) { return (i - 1) / 2; }

    // to get index of left child of node at index i
    int left(int i) { return (2 * i + 1); }

    // to get index of right child of node at index i
    int right(int i) { return (2 * i + 2); }

    // to extract the root which is the minimum element
    MinHeapNode *extractMin();

    // Decreases key value of key at index i to new_val
    void decreaseKey(int i, int new_val);

    // Returns the minimum key (key at root) from min heap
    MinHeapNode *getMin() { return heap[0]; }

    // Deletes a key stored at index i
    void deleteKey(int i);

    // Inserts a new key 'k'
    void insertKey(MinHeapNode *k);

    bool isEmpty() { return size == 0; }

    int getIndexOf(int id);

    void clear();
};
#endif
