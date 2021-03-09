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
    this->has_clusters = false;
    this->first_cluster = nullptr;
    this->number_edges = 0;
    this->node_cont = 0;
    adjacencia = new list<int>[order + 1];
}

Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node, bool has_clusters)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->has_clusters = has_clusters;
    this->first_cluster = nullptr;
    this->number_edges = 0;
    this->node_cont = 0;
    adjacencia = new list<int>;
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

    node_cont++;

    if (node_cont > order)
    {
        order++;
    }
}

Cluster *Graph::getCluster(int id)
{
    Cluster *c = first_cluster;

    while (c != nullptr && c->getId() != id)
    {
        c = c->getNextCluster();
    }

    return c;
}

void Graph::insertNode(int id, int clusterId)
{
    Node *node = new Node(id, clusterId);
    Cluster *c = getCluster(clusterId);

    if (c == nullptr)
    {
        c = insertCluster(clusterId);
    }

    c->insertElement(node);

    if (first_node == nullptr)
    {
        first_node = last_node = node;
    }
    else
    {
        last_node->setNextNode(node);
        last_node = node;
    }

    node_cont++;

    if (node_cont > order)
    {
        order++;
    }
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    Edge edge(id, target_id, weight); //cria aresta com as configura��es dadas
    edges.push_back(edge);            // preenche o vetor de arestas

    if (!has_clusters)
    {
        // a lista de adjacencia e montada adicionando o vertice alvo no array referente ao vertice origem
        adjacencia[id].push_back(target_id);
    }

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

Cluster *Graph::insertCluster(int id)
{
    Cluster *c = new Cluster(id);
    if (first_cluster == nullptr)
    {
        first_cluster = c;
    }
    else
    {
        c->setNextCluster(first_cluster);
        first_cluster = c;
    }
    number_clusters++;

    return c;
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
    int tam = this->getOrder();           //n�mero de n�s visitados
    bool *visitados = new bool[tam];      //vetor que guarda se o v�rtcie foi visitado
    vector<Node *> nos(tam);              //vetor que armazena o endere�o de todos os n�s
    queue<Node *> fila;                   //fila auxilar na ordem de visita��o
    vector<Edge *> tree;                  //�rvore gerada pelo algoritmo de busca em largura
    Node *auxNode = this->getFirstNode(); //n� auxiliar
    Edge *auxEdge;                        //aresta auxiliar
    for (int i = 0; i < tam; i++)
    {
        //iniciando visitados
        *(visitados + i) = false;
        nos[auxNode->getId()] = auxNode;
        auxNode = auxNode->getNextNode();
    }
    fila.push(nos[0]); //inciando a partir do v�rtice de �ndice 0
    *(visitados + fila.front()->getId()) = true;
    while (!fila.empty() && tree.size() != (tam - 1))
    {
        auxNode = fila.front();
        fila.pop();
        auxEdge = auxNode->getFirstEdge();
        int cont = 0;
        while ((cont < (auxNode->getOutDegree() + auxNode->getInDegree())) && auxEdge != nullptr)
        {
            //adiciona todos os n�s visihos n�o visitados
            if (*(visitados + auxEdge->getTargetId()) == false)
            {
                *(visitados + auxEdge->getTargetId()) = true;
                fila.push(nos[auxEdge->getTargetId()]); //atualiza a ordem de inser��o
                Edge *galho = new Edge(auxNode->getId(), auxEdge->getTargetId(), auxEdge->getWeight());
                tree.push_back(galho);
                if (tree.size() == (tam - 1))
                    break;
            }
            auxEdge = auxEdge->getNextEdge();
            cont++;
        }
    }
    float weightResult = 0;
    cout << endl
         << "Busca em Largura" << endl
         << endl;
    if (this->getDirected())
    {
        output_file << "digraph busca{" << endl;
        for (int i = 0; i < tree.size(); i++)
        {
            cout << "(" << tree[i]->getOriginId() << ", " << tree[i]->getTargetId() << ") - peso = " << tree[i]->getWeight() << endl;
            weightResult += tree[i]->getWeight();
            output_file << "\t" << tree[i]->getOriginId() << " -> " << tree[i]->getTargetId() << ";" << endl;
        }
    }
    else
    {
        output_file << "graph busca{" << endl;
        for (int i = 0; i < tree.size(); i++)
        {
            cout << "(" << tree[i]->getOriginId() << ", " << tree[i]->getTargetId() << ") - peso = " << tree[i]->getWeight() << endl;
            weightResult += tree[i]->getWeight();
            output_file << "\t" << tree[i]->getOriginId() << " -- " << tree[i]->getTargetId() << ";" << endl;
        }
    }
    output_file << "}" << endl;
    cout << endl
         << "Peso total da arvore: " << weightResult << endl
         << endl;
}

float Graph::floydMarshall(int idSource, int idTarget, ofstream &output_file)
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
        cout << "Entrada invalida!" << endl;
        return INT_MAX;
    }

    int i, j;

    // uma matriz quadrada de distancias entre os vertices
    float matDistancias[order][order];

    // inicializa a matriz com os maiores valores possiveis
    for (int i = 0; i < order; i++)
    {
        for (int j = 0; j < order; j++)
        {
            matDistancias[i][j] = INT_MAX;
        }
    }

    // variaveis auxiliares para o preenchimento da matriz
    Node *x; //percorrer� os n�s do grafo
    Edge *y; //percorrera as arestas do grafo
    int z;   //indicara o Id do n� de chegada da aresta
    x = first_node;
    y = x->getFirstEdge();
    z = y->getTargetId();

    // preenche a matriz com os valores ja oferecidos
    for (x = first_node; x != nullptr; x = x->getNextNode())
    {
        for (y = x->getFirstEdge(); y != x->getLastEdge(); y = y->getNextEdge())
        {
            z = y->getTargetId();
            matDistancias[x->getId() - 1][z - 1] = y->getWeight();
        }
        y = x->getLastEdge();
        z = y->getTargetId();
        matDistancias[x->getId() - 1][z - 1] = y->getWeight();
    }

    // elementos que representam a distancia de um vertice para ele mesmo inicializados como zero na matriz
    for (int i = 0; i < order; i++)
    {
        for (int j = 0; j < order; j++)
        {
            if (i == j)
                matDistancias[i][j] = 0;
        }
    }

    // matriz de auxilio para impressao do grafo pelo Graphviz
    int matR[order][order];
    for (int i = 0; i < order; i++)
    {
        for (int j = 0; j < order; j++)
        {
            matR[i][j] = j;
        }
    }

    // alagoritmo de Floyd que varre a matriz inicial com apenas os pesos das arestas entre vertices adjacentes e modiifca para o caminho minimo entre dois vertices quaisquer do grafo
    for (int k = 0; k < order; k++)
    {
        for (i = 0; i < order; i++)
        {
            for (j = 0; j < order; j++)
            {
                if (matDistancias[i][j] > matDistancias[i][k] + matDistancias[k][j])
                {
                    matDistancias[i][j] = matDistancias[i][k] + matDistancias[k][j];
                    matR[i][j] = matR[i][k];
                }
            }
        }
    }

    // procedimentos para impressao pelo Graphviz
    int s = idSource - 1;
    int e = idTarget - 1;
    int cont = 0;

    int vet[order];

    for (int i = 0; i < order; i++)
    {
        vet[i] = -1;
    }

    while (s != e)
    {
        vet[cont] = s + 1;
        s = matR[s][e];
        cont++;
    }
    vet[cont] = e + 1;

    output_file << "graph caminho_minimo{" << endl;
    for (int i = 0; vet[i + 1] != -1; i++)
    {
        output_file << vet[i] << " -- " << vet[i + 1] << endl;
    }
    output_file << "}";

    // retorna a distancia entre os dois vertices escolhidos, que estao subtraidos a 1 para serem representados na matriz
    return matDistancias[idSource - 1][idTarget - 1];
}

float Graph::dijkstra(int idSource, int idTarget, ofstream &output_file)
{
    float pi[order];        // vetor que guarda a soma dos custos;
    int prev[order];        // vetor para guardar o id do vertice anterior no caminho minimo;
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
        std::cout << "Entrada inválida!" << endl;
        return INT_MAX;
    }

    // define peso infinito como metade do valor maximo de inteiro, sendo que é metade para evitar overflow
    int infinity = INT_MAX / 2;

    // inicializa s_barra e pi
    node = first_node;
    for (int i = 0; i < order; i++)
    {
        pi[node->getId()] = infinity;
        s_barra.insertKey(new MinHeapNode(node->getId(), infinity));
        node = node->getNextNode();
    }
    pi[idSource] = 0;
    s_barra.decreaseKey(idSource, 0);

    int prevId = -1;
    prev[idSource] = prevId;

    while (!s_barra.isEmpty())
    {
        // Remove o elemento com o menor peso de S-barra
        minNode = s_barra.extractMin();
        prevId = minNode->getId();

        // Procura o elememento extraido na lista do grafo
        for (node = first_node; node->getId() != minNode->getId(); node = node->getNextNode())
            ;

        edge = node->getFirstEdge();
        while (edge != nullptr)
        {
            int pi_estrela = pi[minNode->getId()] + edge->getWeight();

            // Salvando target id em uma varivel pra deixar mais legivell o codigo
            int edgeTargetId = edge->getTargetId();

            if (pi_estrela < pi[edgeTargetId])
            {
                pi[edgeTargetId] = pi_estrela;
                prev[edgeTargetId] = prevId;

                // pega o indice de edgeTargetId na heap de s_barra, caso seja -1, edgeTargetId nao esta em s barra
                int idx = s_barra.getIndexOf(edgeTargetId);
                // se edgeTargetId nao estiver em s_barra, o adcione, se estiver atualize a soma de custos
                if (idx == -1)
                {
                    s_barra.insertKey(new MinHeapNode(edgeTargetId, pi[edgeTargetId]));
                }
                else
                {
                    s_barra.decreaseKey(idx, pi[edgeTargetId]);
                }
            }

            edge = edge->getNextEdge();
        }

        delete minNode;
    }

    int prevNode = idTarget;
    output_file << "graph caminho_minimo{" << endl;

    for (int node = prev[idTarget]; node != -1; node = prev[node])
    {
        output_file << prevNode << " -- " << node << endl;
        prevNode = node;
    }
    output_file << "}";

    return pi[idTarget];
}

//function that prints a topological sorting
void Graph::topologicalSorting(Graph *graph)
{
    stack<int> pilhaTopologica; //pilha que armazena a ordem
    int tamGrafo = graph->getOrder() + 1;
    vector<bool> nosVisitados(tamGrafo, false); // vetor que informa se o vertice ja foi visitado inicializa todas
    // as posicoes como false

    // varredura de todos os vertices, e caso ele nao tenha sido visitado, chama a funcao auxiliar para iniciar a recursao
    for (int i = 0; i < tamGrafo; i++)
    {
        if (nosVisitados[i] == false)
        {
            auxTopologicalSorting(i, nosVisitados, pilhaTopologica);
        }
    }

    cout << "\nOrdenacao topologica:" << endl
         << "< ";

    // estrutura responsavel por imprimir a ordem topologica
    while (pilhaTopologica.empty() == false && pilhaTopologica.top() != 0)
    {
        if (pilhaTopologica.size() > 2)
        {
            cout << pilhaTopologica.top() << ", ";
            pilhaTopologica.pop();
        }
        else
        {
            cout << pilhaTopologica.top() << " ";
            pilhaTopologica.pop();
        }
    }
    cout << ">" << endl
         << endl;
}

//funcao recursiva auxiliar a topologicalSort
void Graph::auxTopologicalSorting(int index, vector<bool> &nosVisitados, stack<int> &pilhaTopologica)
{
    nosVisitados[index] = true; // marco o vertice da vez como visitado

    // busco todos os vertices adjacentes ao index
    list<int>::iterator i;
    for (i = adjacencia[index].begin(); i != adjacencia[index].end(); ++i)
    {
        // verifico se o vertice ja foi visitado e chama a recursao novamente em caso negativo
        if (!nosVisitados[*i])
        {
            auxTopologicalSorting(*i, nosVisitados, pilhaTopologica);
        }
    }

    pilhaTopologica.push(index); // ao final das recursoes, a pilha sera preenchida na ordem em que as chamadas ocorreram
}

void breadthFirstSearch(ofstream &output_file)
{
}

Graph *Graph::getVertexInduced(bool *vertices, int x, ofstream &output_file)
{
    vector<Edge> arestas;
    Graph *g1 = new Graph(x, this->getDirected(), this->getWeightedEdge(), this->getWeightedNode(), has_clusters);
    Node *node = this->getFirstNode();
    Edge *edge;

    for (int i = 0; i < this->getOrder(); i++)
    {
        if (vertices[node->getId()] == true)
        {
            for (edge = node->getFirstEdge(); edge != node->getLastEdge(); edge = edge->getNextEdge())
            {
                if (vertices[edge->getTargetId()] == true)
                {
                    arestas.push_back(Edge(node->getId(), edge->getTargetId(), edge->getWeight()));
                }
            }
            if (vertices[edge->getTargetId()] == true)
            {
                arestas.push_back(Edge(node->getId(), edge->getTargetId(), edge->getWeight()));
            }
            vertices[node->getId()] = false;
            g1->insertNode(node->getId());
        }
        node = node->getNextNode();
    }

    cout << endl
         << "Subgrafo induzido por um conjunto de vertices" << endl
         << endl;
    output_file << "subgraph{" << endl;
    for (int j = 0; j < arestas.size(); j++)
    {
        cout << "(" << arestas[j].getOriginId() << ", " << arestas[j].getTargetId() << ") - peso = " << arestas[j].getWeight() << endl;
        output_file << "\t" << arestas[j].getOriginId() << " -> " << arestas[j].getTargetId() << ";" << endl;

        g1->insertEdge(arestas[j].getOriginId(), arestas[j].getTargetId(), arestas[j].getWeight());
    }
    output_file << "}" << endl;

    return g1;
}

// As funcoes "searchForSubset" e "join" tendem a detectar os ciclos em grafos NAO direcionados. Condicao fundamental
// na montagem do algoritmo de Kruskal.
//essa funcao busca o subconjunto (subset) do no "i" de forma recursiva.
int searchForSubset(int subset[], int i)
{
    if (subset[i] == -1)
        return i;
    return searchForSubset(subset, subset[i]);
}
//a funcao de "join" e unir dois "subsets" (subconjuntos) em 1 unico subconjunto.
void join(int subset[], int v1, int v2)
{
    int v1_set = searchForSubset(subset, v1);
    int v2_set = searchForSubset(subset, v2);
    subset[v1_set] = v2_set;
}
Graph *Graph::agmKuskal(Graph *graph, ofstream &output_file)
{
    vector<Edge> tree; //vetor para armazenar a solucao do problema

    int size_edges = edges.size();

    // Ordena as arestas pelo menor peso.
    sort(edges.begin(), edges.end());

    int V = graph->getOrder();
    int *subset = new int[V + 1];

    //  juntamos todos os subconjuntos em um conjunto proprio. Ex: S={A, B, C, D, E}.
    memset(subset, -1, sizeof(int) * V);

    for (int i = 0; i < size_edges; i++)
    {
        int v1 = searchForSubset(subset, edges[i].getOriginId());
        int v2 = searchForSubset(subset, edges[i].getTargetId());

        // se forem diferentes, sabemos que nao forma ciclo, portanto, inserimos no vetor "tree".
        if (v1 != v2)
        {
            tree.push_back(edges[i]);
            join(subset, v1, v2);
        }
    }

    int size_tree = tree.size();

    // estrutura responsavel por imprimir as arestas selecionadas e seus respectivos pesos, no final, tem-se o custo total.
    cout << endl;
    cout << "Arvore Geradora Minima usando algoritmo de Kruskal" << endl;
    float weightResult = 0;
    for (int i = 0; i < size_tree; i++)
    {
        int v1 = tree[i].getOriginId();
        int v2 = tree[i].getTargetId();
        int w = tree[i].getWeight();
        weightResult = w + weightResult;
        cout << "(" << v1 << ", " << v2 << ") - peso = " << w << endl;
    }
    cout << "Peso total do arvore: " << weightResult << endl;
    cout << endl;

    output_file << "strict graph kruskal{" << endl;
    for (int i = 0; i < size_tree; i++)
    {
        output_file << "\t" << tree[i].getOriginId() << " -- " << tree[i].getTargetId() << ";" << endl;
    }
    output_file << "}";
}
Graph *Graph::agmPrim()
{
    if (!this->getDirected())
    {
        int tam = this->getOrder(); //armazena n�mero de v�rtices.
        int prox[tam];              //armazena o id do v�rtice mais pr�ximo que ainda n�o foi inserido na solu��o
        Graph *tree = new Graph(this->getOrder(), this->getDirected(), this->getWeightedEdge(), this->getWeightedNode());
        vector<Edge> custo; //armazena os menores custos de arestas incidentes na solu��o.
        vector<Node *> nos(tam);
        Node *auxNode = this->getFirstNode();
        int primeiro = auxNode->getId();
        Edge *auxEdge;
        for (int i = 0; i < tam; i++)
        {
            //Inicia o vetor de custo com valores m�ximos e preenche o vetor prox.
            prox[i] = primeiro;
            custo.push_back(Edge(i, primeiro, INT_MAX));
            nos[auxNode->getId()] = auxNode;
            tree->insertNode(auxNode->getId());
            auxNode = auxNode->getNextNode();
        }
        int i, j, k = 0;
        j = (primeiro);
        while (k < tam)
        {
            prox[j] = -1; //atualiza prox.
            auxNode = nos[j];
            auxEdge = auxNode->getFirstEdge();
            int cont = 0;
            while ((cont < (auxNode->getOutDegree() + auxNode->getInDegree())) && auxEdge != nullptr)
            {
                if (prox[auxEdge->getTargetId()] != -1 && (custo[auxEdge->getTargetId()].getWeight() > auxEdge->getWeight()))
                {
                    custo[auxEdge->getTargetId()] = Edge(auxNode->getId(), auxEdge->getTargetId(), auxEdge->getWeight());
                    prox[auxEdge->getTargetId()] = auxNode->getId();
                }
                auxEdge = auxEdge->getNextEdge();
                cont++;
            }
            //encontra a aresta j que n�o faz parte da solu��o e tem o menor peso.
            for (i = 0; i < tam; i++)
                if (prox[i] != -1)
                {
                    j = i;
                    break;
                }
            for (; i < tam; i++)
                if (prox[i] != -1 && custo[i].getWeight() < custo[j].getWeight())
                    j = i;
            if (custo[j].getWeight() == INT_MAX)
            {
                cout << "Arvore Geradora Minima usando algoritmo de Prim" << endl
                     << endl;
                cout << "Grafo desconexo" << endl
                     << endl;
                return nullptr;
            }
            k++;
        }
        sort(custo.begin(), custo.end());
        cout << "Arvore Geradora Minima usando algoritmo de Prim" << endl
             << endl;
        float weightResult = 0;
        for (int i = 0; i < tam - 1; i++)
        {
            //Imprime a solu��o
            cout << "(" << custo[i].getOriginId() << ", " << custo[i].getTargetId() << ") - peso = " << custo[i].getWeight() << endl;
            weightResult += custo[i].getWeight();
            tree->insertEdge(custo[i].getOriginId(), custo[i].getTargetId(), custo[i].getWeight());
        }
        cout << endl
             << "Peso total da arvore: " << weightResult << endl
             << endl;
        return tree;
    }
    else
    {
        cout << "Arvore Geradora Minima usando algoritmo de Prim" << endl
             << endl;
        cout << "Grafo direcionado" << endl
             << endl;
        return nullptr;
    }
}

void addEdges(vector<Edge *> *vet, Node *sourceNode, Graph *g, bool visitedClusters[])
{
    Edge *edge = g->getNode(sourceNode->getId())->getFirstEdge();

    while (edge != nullptr)
    {
        if (!visitedClusters[g->getNode(edge->getTargetId())->getCluster() - 1])
            vet->push_back(edge);
        edge = edge->getNextEdge();
    }
}

// Algoritmo guloso para o problema de otmizaçao da árvore geradora mínima generalizada
// Adaptaçao de PRIM
Graph *Graph::greed()
{
    if (!this->has_clusters)
    {
        cout << "Erro: Grafo nao tem grupos para se realizar arvore minima generalizada. Tente o algoritmo de Prim ou de Kruskal." << endl;
        return nullptr;
    }

    Graph *minimalTree = nullptr;          // Armazena a menor arvore entre todas iteraçoes
    Graph *tree;                           // armazena arvore construida em determinada iteraçao
    int minCost = INT_MAX;                 // guarda o menor custo entre todas iteracoes
    int currentCost;                       // usado para calcular custo em determinada itecao
    Cluster *cluster;                      // grupo do qual esta partindo o algoritmo em determina iteracao
    vector<Edge *> k(number_edges);        // guarda arestas candidatas
    bool visitedClusters[number_clusters]; // armazena quais grupos ja foram visitados
    Node *node;                            // Vertice auxiliar
    Edge *edge;                            // Aresta auxiliar

    // O resultado pode alterar dependo de qual grupo se começa, entao o algoritmo se executa varias vezes, cada vez começando de um grupo
    for (cluster = first_cluster; cluster != nullptr; cluster = cluster->getNextCluster())
    {
        tree = new Graph(0, directed, weighted_edge, weighted_node, has_clusters); // inicializa arvore vazia pra guardar o resultado da iteracao

        for (int i = 0; i < number_clusters; i++)
        {
            visitedClusters[i] = false; // marca todos os grupo como nao visitados
        }
        currentCost = 0;
        visitedClusters[cluster->getId() - 1] = true; // marca o grupo atual como visitado nao entrarem arestas internas como candidatas

        // Pecorre todos vertices do grupo e adciona qualquer aresta pra qualquer outro grupo como candidata
        for (int i = 0; i < cluster->getSize(); i++)
        {
            addEdges(&k, cluster->getElement(i), this, visitedClusters);
        }

        // Como ha um grupo ja visitado, precisa-se de n - 1 iteracoes para visitar todos os grupos, sendo n a quantidade de grupos
        for (int i = 1; i < number_clusters; i++)
        {
            // Monta o conjuto de arestas candidatas a partir das adjacencias dos vertices que ja pertencem a solucao
            for (node = tree->getFirstNode(); node != nullptr; node = node->getNextNode())
            {
                addEdges(&k, node, this, visitedClusters);
            }

            // Procura aresta com o menor custo
            edge = k[0];
            for (int i = 1; i < k.size(); i++)
            {
                if (k[i]->getWeight() < edge->getWeight())
                {
                    edge = k[i];
                }
            }

            if (!tree->searchNode(edge->getOriginId()))
            {
                tree->insertNode(edge->getOriginId());
            }
            if (!tree->searchNode(edge->getTargetId()))
            {
                tree->insertNode(edge->getTargetId());
            }
            tree->insertEdge(edge->getOriginId(), edge->getTargetId(), edge->getWeight()); // adciona a aresta de menor custo na solucao
            currentCost += edge->getWeight();                                              // atualiza custo da solucao
            visitedClusters[this->getNode(edge->getTargetId())->getCluster() - 1] = true;  // marca o grupo do vertice alvo da aresta como visitado

            k.clear(); // limpa conjunto de arestas candidatas para ser reconstruido na proxima iteracao
        }

        // Se a solucao dessa iteracao for a melhor ate o momento, guarde ela
        if (currentCost < minCost)
        {
            if (minimalTree != nullptr)
            {
                delete minimalTree;
            }
            minimalTree = tree;
            minCost = currentCost;
        }
        else
        {
            // Se nao for a melhor solucao, apague-a;
            delete tree;
        }
    }

    std::cout << "Custo total da árvore: " << minCost << endl;
    return minimalTree;
}

bool compareEdgesWeight(Edge *a, Edge *b)
{
    return a->getWeight() < b->getWeight();
}

Graph *Graph::greedRandom()
{
    if (!this->has_clusters)
    {
        std::cout << "Erro: Grafo nao tem grupos para se realizar arvore minima generalizada. Tente o algoritmo de Prim ou de Kruskal." << endl;
        return nullptr;
    }

    Graph *minimalTree = nullptr;          // Armazena a menor arvore entre todas iteraçoes
    Graph *tree;                           // armazena arvore construida em determinada iteraçao
    int minCost = INT_MAX;                 // guarda o menor custo entre todas iteracoes
    int currentCost;                       // usado para calcular custo em determinada itecao
    Cluster *cluster;                      // grupo do qual esta partindo o algoritmo em determina iteracao
    vector<Edge *> k(number_edges);        // guarda arestas candidatas
    float alpha = 0.05;                    // indice de randomizacao
    float max;                             // O mairo custo aceitavel na randomizacao
    bool visitedClusters[number_clusters]; // armazena quais grupos ja foram visitados
    Node *node;                            // Vertice auxiliar
    Edge *edge;                            // Aresta auxiliar

    srand(time(NULL));

    // O resultado pode alterar dependo de qual grupo se começa, entao o algoritmo se executa varias vezes, cada vez começando de um grupo
    for (cluster = first_cluster; cluster != nullptr; cluster = cluster->getNextCluster())
    {
        tree = new Graph(0, directed, weighted_edge, weighted_node, has_clusters); // inicializa arvore vazia pra guardar o resultado da iteracao

        for (int i = 0; i < number_clusters; i++)
        {
            visitedClusters[i] = false; // marca todos os grupo como nao visitados
        }
        currentCost = 0;
        visitedClusters[cluster->getId() - 1] = true; // marca o grupo atual como visitado nao entrarem arestas internas como candidatas

        // Pecorre todos vertices do grupo e adciona qualquer aresta pra qualquer outro grupo como candidata
        for (int i = 0; i < cluster->getSize(); i++)
        {
            addEdges(&k, cluster->getElement(i), this, visitedClusters);
        }

        // Como ha um grupo ja visitado, precisa-se de n - 1 iteracoes para visitar todos os grupos, sendo n a quantidade de grupos
        for (int i = 1; i < number_clusters; i++)
        {
            // Monta o conjuto de arestas candidatas a partir das adjacencias dos vertices que ja pertencem a solucao
            for (node = tree->getFirstNode(); node != nullptr; node = node->getNextNode())
            {
                addEdges(&k, node, this, visitedClusters);
            }
            sort(k.begin(), k.end(), compareEdgesWeight); // ordena as arestas candidatas
            max = k.front()->getWeight() + alpha * (k.back()->getWeight() - k.front()->getWeight());

            for (int i = 0; i < k.size(); i++)
            {
                if (k[i]->getWeight() == max)
                {
                    edge = k[rand() % (i + 1)];
                    break;
                }
                else if (k[i]->getWeight() > max)
                {
                    edge = k[rand() % i];
                    break;
                }
            }

            if (!tree->searchNode(edge->getOriginId()))
            {
                tree->insertNode(edge->getOriginId());
            }
            if (!tree->searchNode(edge->getTargetId()))
            {
                tree->insertNode(edge->getTargetId());
            }
            tree->insertEdge(edge->getOriginId(), edge->getTargetId(), edge->getWeight()); // adciona a aresta de menor custo na solucao
            currentCost += edge->getWeight();                                              // atualiza custo da solucao
            visitedClusters[this->getNode(edge->getTargetId())->getCluster() - 1] = true;  // marca o grupo do vertice alvo da aresta como visitado

            k.clear(); // limpa conjunto de arestas candidatas para ser reconstruido na proxima iteracao
        }

        // Se a solucao dessa iteracao for a melhor ate o momento, guarde ela
        if (currentCost < minCost)
        {
            if (minimalTree != nullptr)
            {
                delete minimalTree;
            }
            minimalTree = tree;
            minCost = currentCost;
        }
        else
        {
            delete tree; // Se nao for a melhor solucao, apague-a;
        }
    }

    std::cout << "Custo total da árvore: " << minCost << endl;
    return minimalTree;
}
