/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include "Edge.h"
#include <fstream>
#include <stack>
#include <list>

using namespace std;

class Graph
{

    //Atributes
private:
    int order;
    int number_edges;
    int number_groups;
    bool directed;
    bool weighted_edge;
    bool weighted_node;
    int node_cont;
    Node *first_node;
    Node *last_node;

public:
    //Constructor
    Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
    //Destructor
    ~Graph();
    //Getters
    int getOrder();
    int getNumberEdges();
    bool getDirected();
    bool getWeightedEdge();
    bool getWeightedNode();
    Node *getFirstNode();
    Node *getLastNode();
    //Other methods
    void insertNode(int id);
    void insertNode(int id, int group);
    void insertEdge(int id, int target_id, float weight);
    void removeNode(int id);
    bool searchNode(int id);
    Node *getNode(int id);

    //methods phase1
    void topologicalSorting();
    void breadthFirstSearch(ofstream &output_file);
    Graph *getVertexInduced(int *listIdNodes, Graph &graph, int x);
    Graph *agmKuskal(Graph *graph);
    Graph *agmPrim();
    float floydMarshall(int idSource, int idTarget);
    float dijkstra(int idSource, int idTarget, ofstream &output_file);

    //methods phase1
    float greed();
    float greedRandom();
    float greedRactiveRandom();

private:
};

#endif // GRAPH_H_INCLUDED
