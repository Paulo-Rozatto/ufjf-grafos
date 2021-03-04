#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <tuple>
#include <iomanip>
#include <stdlib.h>
#include <chrono>
#include "Graph.h"
#include "Node.h"

using namespace std;

Graph *leitura(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;


    //Pegando a ordem do grafo
    input_file >> order;

    //Criando objeto grafo
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo


    if (!graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        while (input_file >> idNodeSource >> idNodeTarget )
        {
            if (!graph->searchNode(idNodeSource))
                graph->insertNode(idNodeSource);
            if (!graph->searchNode(idNodeTarget))
                graph->insertNode(idNodeTarget);
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
        }
    }
    else if (graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        float edgeWeight;

        while (input_file >> idNodeSource >> idNodeTarget >> edgeWeight)
        {

            if (!graph->searchNode(idNodeSource))
                graph->insertNode(idNodeSource);
            if (!graph->searchNode(idNodeTarget))
                graph->insertNode(idNodeTarget);
            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
        }
    }
    else if (graph->getWeightedNode() && !graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight)
        {
            if (!graph->searchNode(idNodeSource))
                graph->insertNode(idNodeSource);
            if (!graph->searchNode(idNodeTarget))
                graph->insertNode(idNodeTarget);
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }
    else if (graph->getWeightedNode() && graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight, edgeWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight >> edgeWeight)
        {
            if (!graph->searchNode(idNodeSource))
                graph->insertNode(idNodeSource);
            if (!graph->searchNode(idNodeTarget))
                graph->insertNode(idNodeTarget);
            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }

    return graph;
}

Graph *leituraInstancia(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;
    int numEdges;

    //Pegando a ordem do grafo
    input_file >> order >> numEdges;

    //Criando objeto grafo
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo
    while (input_file >> idNodeSource >> idNodeTarget)
    {
        if (!graph->searchNode(idNodeSource))
            graph->insertNode(idNodeSource);
        if (!graph->searchNode(idNodeTarget))
            graph->insertNode(idNodeTarget);
        graph->insertEdge(idNodeSource, idNodeTarget, 0);
    }

    return graph;
}

void escrita(Graph *graph, ofstream &output_file){
    if(graph != nullptr){
        Node *auxNode = graph->getFirstNode();
        Edge *auxEdge;
        int tam = graph->getOrder();
        if(graph->getDirected()){
            output_file << "strict digraph grafo{" << endl;
            for(int i = 0; i < tam; i++){
                auxEdge = auxNode->getFirstEdge();
                int cont = 0;
                while((cont < (auxNode->getOutDegree() + auxNode->getInDegree())) && auxEdge != nullptr){
                    output_file << "\t" <<  auxNode->getId() << " -> " << auxEdge->getTargetId() << ";" << endl;
                    auxEdge = auxEdge->getNextEdge();
                    cont ++;
                }
                auxNode = auxNode->getNextNode();
            }
            output_file << "}" << endl;
        }
        else{
           output_file << "strict graph grafo{" << endl;
            for(int i = 0; i < tam; i++){
                auxEdge = auxNode->getFirstEdge();
                int cont = 0;
                while((cont < (auxNode->getOutDegree() + auxNode->getInDegree())) && auxEdge != nullptr){
                    output_file << "\t" <<  auxNode->getId() << " -- " << auxEdge->getTargetId() << ";" << endl;
                    auxEdge = auxEdge->getNextEdge();
                    cont ++;
                }
                auxNode = auxNode->getNextNode();
            }
            output_file << "}" << endl;
        }
    }
}

int menu()
{

    int selecao;

    cout << "MENU" << endl;
    cout << "----" << endl;
    cout << "[1] Subgrafo induzido por conjunto de vertices" << endl;
    cout << "[2] Caminho Minimo entre dois vertices - Dijkstra" << endl;
    cout << "[3] Caminho Minimo entre dois vertices - Floyd" << endl;
    cout << "[4] Arvore Geradora Minima de Prim" << endl;
    cout << "[5] Arvore Geradora Minima de Kruskal" << endl;
    cout << "[6] Imprimir caminhamento em largura" << endl;
    cout << "[7] Imprimir ordenacao topologica" << endl;
    cout << "[8] Algoritmo Guloso" << endl;
    cout << "[9] Algoritmo Guloso Randomizado " << endl;
    cout << "[10] Algoritmo Guloso Randomizado Reativo" << endl;
    cout << "[0] Sair" << endl;

    cin >> selecao;

    return selecao;
}

void selecionar(int selecao, Graph *graph, ofstream &output_file)
{

    switch (selecao)
    {

    //Subgrafo induzido por um conjunto de vértices X;
    case 1:
    {
        int x;
        cout << "Quantos vertices voce quer digitar?" << endl;
        cin >> x;
        int vertices[x];
        cout << "Digite os vertices: ";
        for(int i = 0; i < x; i++)
        {
            cin >> vertices[i];
        }
        Graph *g = g->getVertexInduced(vertices, *graph, x);
        Node *node = graph->getFirstNode();
        cout << "vertices grafo original: ";

        while(node != nullptr)
        {
            cout << node->getId();
            node = node->getNextNode();
        }
        cout << endl;
        cout << "vertices grafo induzido: ";
        for(Node *node = g->getFirstNode(); node != nullptr; node = node->getNextNode())
        {
            cout << node->getId();
        }
        cout << endl;
        break;
    }
        //Caminho mínimo entre dois vértices usando Dijkstra;
    case 2:
    {
        int source, target;
        cout << "Caminho mínimo entre dois vértices usando Dijkstra;" << endl;
        cout << "Digite o id do vértice de partida: ";
        cin >> source;
        cout << "Digite o id do vértice de destino: ";
        cin >> target;
        cout << "Comprimento do caminho minimo: " << graph->dijkstra(source, target) << endl;
        break;
    }

        //Caminho mínimo entre dois vértices usando Floyd;
    case 3:
    {
        int source, target;
        cout << "Caminho minimo entre dois vertices usando Floyd;" << endl;
        cout << "Digite o id do vertice de partida: ";
        cin >> source;
        cout << "Digite o id do vertice de destino: ";
        cin >> target;
        cout << "Comprimento do caminho minimo entre os vertices escolhidos: ";
        cout << graph->floydMarshall(source, target) << endl;
        break;
    }


    case 4:
    {

        Graph *tree = graph->agmPrim();
        escrita(tree, output_file);
        break;
    }

        //AGM Prim;
    case 5:
    {
        graph->agmKuskal(graph);
        break;
    }



        //Busca em largura;
    case 6:
    {
        graph->breadthFirstSearch(output_file);
        break;
    }
        //Ordenação Topologica;
    case 7:
    {
        graph->topologicalSorting(graph);
        break;
    }
    case 0:
    {
        cout << "Execution finished" << endl;
        break;
    }
    default:
    {
        cout << " Error!!! invalid option!!" << endl;
    }
    }
}

int mainMenu(ofstream &output_file, Graph *graph)
{

    int selecao = 1;

    while (selecao != 0)
    {
        // system("clear");
        selecao = menu();

        if (output_file.is_open())
            selecionar(selecao, graph, output_file);

        else
            cout << "Unable to open the output_file" << endl;

        output_file << endl;
    }

    return 0;
}

int main(int argc, char const *argv[])
{

    //Verificação se todos os parâmetros do programa foram entrados
    if (argc != 6)
    {

        cout << "ERROR: Expecting: ./<program_name> <input_file> <output_file> <directed> <weighted_edge> <weighted_node> " << endl;
        return 1;
    }

    string program_name(argv[0]);
    string input_file_name(argv[1]);

    string instance;
    if (input_file_name.find("v") <= input_file_name.size())
    {
        string instance = input_file_name.substr(input_file_name.find("v"));
        cout << "Running " << program_name << " with instance " << instance << " ... " << endl;
    }

    //Abrindo arquivo de entrada
    ifstream input_file;
    ofstream output_file;
    input_file.open(argv[1], ios::in);
    output_file.open(argv[2], ios::out | ios::trunc);

    Graph *graph;

    if (input_file.is_open())
    {
        graph = leitura(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
    }
    else
        cout << "Unable to open " << argv[1];

    mainMenu(output_file, graph);

    //Fechando arquivo de entrada
    input_file.close();

    //Fechando arquivo de saída
    output_file.close();

    return 0;
}
