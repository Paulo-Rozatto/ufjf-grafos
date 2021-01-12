#include <iostream>
#include <fstream>

using namespace std;

void imprimeArquivoDeEntrada(string nomeArquivo)
{
    ifstream arquivo;
    string texto;

    arquivo.open(nomeArquivo); // tenta abrir um arquivo de texto

    if(arquivo.is_open() && arquivo.good()) // verifica se abertura do arquivo deu certo
    {

        while (getline (arquivo, texto))
        {
            cout << texto << endl;
        }

        arquivo.close();
    }
    else
    {
        cout << "Nenhum arquivo de texto valido encontrado." << endl;
    }
}

/**
* @param argc - quantidade de argumentos passado para main via terminal de linha de comando
* @param *argv[] - vetor de char que contém os argumentos, um para cada string passada na linha de comando.
*
* argv[0] armazena o nome do programa que foi chamado no prompt, sendo assim, argc é pelo menos igual a 1, pois no mínimo existirá um argumento.
*/
int main(int argc, char* argv[])
{
    cout << argc << " argumentos passados.\n";

    for (int i = 0; i < argc; ++i)
    {
        cout << argv[i] << "\n";
    }

    if(argc > 1)
    {
        cout << "\nTentando abrir " << argv[1] << endl;
        imprimeArquivoDeEntrada(argv[1]);
    }

    return 0;
}
