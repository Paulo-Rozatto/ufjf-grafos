#ifndef MIN_HEAP_NODE_H
#define MIN_HEAP_NODE_H

class MinHeapNode
{
private:
    int id;
    float weight;

public:
    MinHeapNode(){};
    MinHeapNode(int id, float weight)
    {
        this->id = id;
        this->weight = weight;
    };
    ~MinHeapNode(){};
    int getId() { return id; };
    float getWeight() { return weight; };
    void setWeight(int weight) { this->weight = weight; }
};
#endif
