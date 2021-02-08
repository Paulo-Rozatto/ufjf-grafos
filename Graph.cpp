#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include "MinHeap.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>
#include <climits>

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
}

// Destructor
Graph::~Graph()
{

    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder()
{

    return this->order;
}
int Graph::getNumberEdges()
{

    return this->number_edges;
}
//Function that verifies if the graph is directed
bool Graph::getDirected()
{

    return this->directed;
}
//Function that verifies if the graph is weighted at the edges
bool Graph::getWeightedEdge()
{

    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode()
{

    return this->weighted_node;
}

Node *Graph::getFirstNode()
{

    return this->first_node;
}

Node *Graph::getLastNode()
{

    return this->last_node;
}

// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node *node = new Node(id);

    if (first_node == nullptr)
    {
        first_node = last_node = node;
    }
    else
    {
        last_node->setNextNode(node);
        last_node = node;
    }

    // order++;
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    Node *node, *target_node;

    node = getNode(id);
    // try to get target_node only if node exists
    if (node != nullptr)
    {
        target_node = getNode(target_id);

        // inserts edge only if target_node also exists
        if (target_node != nullptr)
        {
            node->insertEdge(target_id, weight);

            if (directed)
            {
                node->incrementOutDegree();
                target_node->incrementInDegree();
            }
            else
            {
                node->incrementInDegree();
                target_node->insertEdge(id, weight);
                target_node->incrementOutDegree();
            }
        }
    }
}

void Graph::removeNode(int id)
{
}

bool Graph::searchNode(int id)
{
    Node *node = first_node;

    while (node != nullptr)
    {
        if (node->getId() == id)
        {
            return true;
        }

        node = node->getNextNode();
    }

    return false;
}

Node *Graph::getNode(int id)
{
    Node *node = first_node;

    while (node != nullptr && node->getId() != id)
    {
        node = node->getNextNode();
    }

    return node;
}

//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file)
{
}

float Graph::floydMarshall(int idSource, int idTarget)
{
}

float Graph::dijkstra(int idSource, int idTarget)
{
    float pi[order];        // vetor que guarda a soma dos custos
    MinHeap s_barra(order); // Heap minima para ser S-barra
    MinHeapNode *minNode;   // No da heap que guarda id do vertice e soma dos custos;
    Node *node;
    Edge *edge;

    // verifica se os vertices passados por paramentro existem no grafo
    bool isSource = false, isTarget = false;
    for (node = first_node; node != nullptr; node = node->getNextNode())
    {
        if (node->getId() == idSource)
            isSource = true;
        if (node->getId() == idTarget)
            isTarget = true;
    }
    // se pelo menos um dos vertices passados por parametro nao existir no grafo, retorna infinito e exibe mensagem
    if (!isSource || !isTarget)
    {
        cout << "Entrada inválida!" << endl;
        return INT_MAX;
    }

    // inicializa s_barra e pi
    node = first_node;
    for (int i = 0; i < order; i++)
    {
        if (node->getId() == idSource)
        {
            pi[i] = 0;
            s_barra.insertKey(new MinHeapNode(node->getId(), 0));
        }
        else
        {
            pi[i] = INT_MAX;
            s_barra.insertKey(new MinHeapNode(node->getId(), INT_MAX));
        }
        node = node->getNextNode();
    }

    while (!s_barra.isEmpty())
    {
        // Remove o elemento com o menor peso de S-barra
        minNode = s_barra.extractMin();

        // Procura o elememento extraido na lista do grafo e salva o seu indice na variavel j para ser usado no vetor de distancias
        int j = 0;
        for (node = first_node; node->getId() != minNode->getId(); node = node->getNextNode())
            j++;

        edge = node->getFirstEdge();
        while (edge != nullptr)
        {
            int pi_estrela = pi[j] + edge->getWeight();

            // Procura o vertice adjacente a j na lista e salva seu indice na variavel k
            int k = 0;
            for (node = first_node; node->getId() != edge->getTargetId(); node = node->getNextNode())
                k++;

            if (pi_estrela < pi[k])
            {
                pi[k] = pi_estrela;

                // pega o indice de k na heap de s_barra, caso seja -1, k nao esta em s barra
                int idx = s_barra.getIndexOf(node->getId());
                // se k nao estiver em s_barra, o adcione, se estiver atualize a soma de custos
                if (idx == -1)
                {
                    s_barra.insertKey(new MinHeapNode(node->getId(), pi[k]));
                }
                else
                {
                    s_barra.decreaseKey(idx, pi[k]);
                }
            }

            edge = edge->getNextEdge();
        }

        delete minNode;
    }

    int i = 0;
    for (node = first_node; node->getId() != idTarget; node = node->getNextNode())
        i++;

    return pi[i];
}

//function that prints a topological sorting
void topologicalSorting()
{
}

void breadthFirstSearch(ofstream &output_file)
{
}
Graph *getVertexInduced(int *listIdNodes)
{
}

Graph *agmKuskal()
{
}
Graph *agmPrim()
{
}
