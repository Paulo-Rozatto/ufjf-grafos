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
#include <algorithm>
#include <string.h>
#include <vector>
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

    vector<Edge> edges; //vetor das arestas

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

//    order++;

}

void Graph::insertEdge(int id, int target_id, float weight)
{
    Edge edge(id, target_id, weight); //cria aresta com as configura��es dadas
    edges.push_back(edge); // preenche o vetor de arestas


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

    int i, j;

    // uma matriz quadrada de distancias entre os vertices
    float matDistancias[order][order];

    // elementos que representam a distancia de um vertice para ele mesmo inicializados como zero na matriz
    for(int i = 0; i < order; i++)
    {
        for(int j = 0; j < order; j++)
        {
            if(i == j)
                matDistancias[i][j] = 0;
        }
    }

    // algoritmo de leitura nao pega um terceiro parametro (peso) de entrada.txt; Aqui se faz necessaria a inserçao manual das arestas:
    matDistancias[0][1]= 7;
    matDistancias[0][2]= 1;
    matDistancias[0][3]= INT_MAX;
    matDistancias[0][4]= INT_MAX;
    matDistancias[0][5]= INT_MAX;
    matDistancias[1][0]= 7;
    matDistancias[1][2]= 5;
    matDistancias[1][3]= 4;
    matDistancias[1][4]= 2;
    matDistancias[1][5]= 1;
    matDistancias[2][0]= 1;
    matDistancias[2][1]= 5;
    matDistancias[2][3]= INT_MAX;
    matDistancias[2][4]= 2;
    matDistancias[2][5]= 7;
    matDistancias[3][0]= INT_MAX;
    matDistancias[3][1]= 4;
    matDistancias[3][2]= INT_MAX;
    matDistancias[3][4]= 5;
    matDistancias[3][5]= INT_MAX;
    matDistancias[4][0]= INT_MAX;
    matDistancias[4][1]= 2;
    matDistancias[4][2]= 2;
    matDistancias[4][3]= 5;
    matDistancias[4][5]= 3;
    matDistancias[5][0]= INT_MAX;
    matDistancias[5][1]= 1;
    matDistancias[5][2]= 7;
    matDistancias[5][3]= INT_MAX;
    matDistancias[5][4]= 3;

    // alagoritmo de Floyd que varre a matriz inicial com apenas os pesos das arestas entre vertices adjacentes e modiifca para o caminho minimo entre dois vertices quaisquer do grafo
    for (int k=0; k<order; k++)
    {
        for (i=0; i<order; i++)
        {
             for (j=0; j<order; j++)
             {
                  if (matDistancias[i][j] > matDistancias[i][k]+matDistancias[k][j])
                    matDistancias[i][j] = matDistancias[i][k]+matDistancias[k][j];
             }
        }
    }
    // retorna a distancia entre os dois vertices escolhidos, que estao subtraidos a 1 para serem representados na matriz
    return matDistancias[idSource-1][idTarget-1];

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

Graph* Graph::getVertexInduced(int *listIdNodes, Graph &graph, int x)
{
    //cria uma copia do grafo original para serem feitas as opera��es
    Graph *g1 = new Graph(graph);
    Node *node = graph.getFirstNode();

    //percorre o grafo em busca dos nos passados pelo array por parametro
    while(node != nullptr)
    {
        bool verifica = false;

        for(int i = 0; i < x; i++)
        {
            if(node->getId() == listIdNodes[i])
            {
                verifica = true;
            }
        }
        //exclui o n� do grafo copia para transformar ele em subgrafo induzido
        if(!verifica)
        {
            g1->removeNode(node->getId());
        }
        node = node->getNextNode();
    }
    //retorna o subgrafo induzido
    return g1;

}

/// As fun��es "searchForSubset" e "join" tendem a detectar os ciclos em grafos N�O direcionados. Condi��o fundamental
/// na montagem do algoritmo de Kruskal.
//essa fun��o busca o subconjunto (subset) do n� "i" de forma recursiva.
int searchForSubset(int subset[], int i){
    if(subset[i] == -1)
        return i;
    return searchForSubset(subset, subset[i]);
}
//a fun��o de "join" � unir dois "subsets" (subconjuntos) em 1 �nico subconjunto.
void join(int subset[], int v1, int v2){
    int v1_set = searchForSubset(subset, v1);
    int v2_set = searchForSubset(subset, v2);
    subset[v1_set] = v2_set;
}

Graph *Graph::agmKuskal(Graph *graph){
    vector<Edge> tree; //vetor para armazenar a solu��o do problema

    int size_edges = edges.size();


// Ordena as arestas pelo menor peso.
    sort(edges.begin(), edges.end());

    int V = graph->getOrder();
    int * subset = new int[V+1];

//  juntamos todos os subconjuntos em um conjunto pr�prio. Ex: S={A, B, C, D, E}.
    memset(subset, -1, sizeof(int) * V);

    for(int i = 0; i < size_edges; i++){
        int v1 = searchForSubset(subset, edges[i].getOriginId());
        int v2 = searchForSubset(subset, edges[i].getTargetId());

// se forem diferentes, sabemos que n�o forma ciclo, portanto, inserimos no vetor "tree".
        if(v1 != v2){
            tree.push_back(edges[i]);
            join(subset, v1, v2);
        }
    }

    int size_tree = tree.size();

// tem a fun��o de mostrar as arestas selecionadas e seus respectivos pesos, no final, tem-se o custo total.
    cout << endl;
    cout << "Arvore Geradora Minima usando algoritmo de Kruskal" << endl;
    float weightResult = 0;
    for(int i = 0; i < size_tree; i++){
        int v1 = tree[i].getOriginId();
        int v2 = tree[i].getTargetId();
        int w = tree[i].getWeight();
        weightResult = w + weightResult;
        cout << "(" << v1 << ", " << v2 << ") - peso = " << w << endl;
    }
    cout << "Peso total do arvore: " << weightResult << endl;
    cout << endl;

}

Graph *agmPrim()
{
}
