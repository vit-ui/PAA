#include <iostream>
#include "./dijsktra.cpp"
#include "./helpers.cpp"
int main(){
    int tamanho = 3;
    for(int i = 0; i < 10; i++)
    {
        if(i % 2 == 0) tamanho++;

        auto grafo = geraGrafo(tamanho, 1);
        salvaGrafo(tamanho, 0.7, grafo);
    }
    imprimeArquivo("../grafos/grafos.txt");
}