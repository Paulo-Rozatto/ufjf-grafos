#include <climits>
#include <iostream>
#include "MinHeap.h"
#include "MinHeapNode.h"

// A utility function to swap two elements
void swap(MinHeapNode **x, MinHeapNode **y)
{
    MinHeapNode *temp = *x;
    *x = *y;
    *y = temp;
}

// Constructor: Builds a heap from a given array a[] of given size
MinHeap::MinHeap(int capacity)
{
    size = 0;
    this->capacity = capacity;
    heap = new MinHeapNode *[capacity];
}

MinHeap::~MinHeap()
{
    for (int i = 0; i < size; i++)
    {
        delete heap[i];
    }
    delete[] heap;
}

// Inserts a new key 'k'
void MinHeap::insertKey(MinHeapNode *k)
{
    if (size == capacity)
    {
        std::cout << "Heap is full, key not inserted" << std::endl;
        return;
    }

    // First insert the new key at the end
    int i = size;
    heap[i] = k;
    size++;

    // Fix the min heap property if it is violated
    while (i != 0 && heap[parent(i)]->getWeight() > heap[i]->getWeight())
    {
        swap(&heap[i], &heap[parent(i)]);
        i = parent(i);
    }
}

// Decreases value of key at index 'i' to new_val. It is assumed that
// new_val is smaller than harr[i].
void MinHeap::decreaseKey(int i, int new_val)
{
    heap[i]->setWeight(new_val);
    while (i != 0 && heap[parent(i)]->getWeight() > heap[i]->getWeight())
    {
        swap(&heap[i], &heap[parent(i)]);
        i = parent(i);
    }
}

// Method to remove minimum element (or root) from min heap
MinHeapNode *MinHeap::extractMin()
{
    if (size <= 0)
        return nullptr;
    if (size == 1)
    {
        size--;
        return heap[0];
    }

    // Store the minimum value, and remove it from heap
    MinHeapNode *root = heap[0];
    heap[0] = heap[size - 1];
    size--;
    MinHeapify(0);

    return root;
}

// This function deletes key at index i. It first reduced value to minus
// infinite, then calls extractMin()
void MinHeap::deleteKey(int i)
{
    decreaseKey(i, INT_MIN);
    extractMin();
}

// A recursive method to heapify a subtree with the root at given index
// This method assumes that the subtrees are already heapified
void MinHeap::MinHeapify(int i)
{
    int l = left(i);
    int r = right(i);
    int smallest = i;
    if (l < size && heap[l] < heap[i])
        smallest = l;
    if (r < size && heap[r] < heap[smallest])
        smallest = r;
    if (smallest != i)
    {
        swap(&heap[i], &heap[smallest]);
        MinHeapify(smallest);
    }
}

int MinHeap::getIndexOf(int id)
{
    for (int i = 0; i < size; i++)
    {
        if (heap[i]->getId() == id)
            return i;
    }
    return -1;
}

void MinHeap::clear()
{
    for (int i = 0; i < size; i++)
    {
        delete heap[i];
    }
    size = 0;
}