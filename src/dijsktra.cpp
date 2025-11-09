#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <utility>
#include <iostream>

const int INFINITO = INT_MAX;
using Grafo = std::vector<std::vector<std::pair<int, int>>>;

std::pair<std::vector<int>, std::vector<int>> dijkstra(const std::vector<std::vector<std::pair<int, int>>>& grafo){
    int tamanho = grafo.size();

    std::vector<int> minDistancia(tamanho, INFINITO); //dt
    std::vector<int> predecessores(tamanho, -1); //rot

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> verticesParaProcessar;
    minDistancia[0] = 0;
    verticesParaProcessar.push({0,0});

    while(!verticesParaProcessar.empty()){
        auto parAtual = verticesParaProcessar.top();
        verticesParaProcessar.pop();

        int distancia = parAtual.first;
        int verticeAtual = parAtual.second;
        if (distancia > minDistancia[verticeAtual]) continue;

        for(const auto& parVizinho : grafo[verticeAtual]){
            int vizinho = parVizinho.first;
            int peso = parVizinho.second;

            // linha 14 era redundante. Pulei direto para 15.
            int distanciaNova = minDistancia[verticeAtual] + peso;
            if(distanciaNova < minDistancia[vizinho]){
                minDistancia[vizinho] = distanciaNova;
                predecessores[vizinho] = verticeAtual;
                verticesParaProcessar.push({distanciaNova, vizinho});
            }
        }
    }
    return std::make_pair(minDistancia, predecessores);
}

int main() {
    
    // 1. Define o tamanho do nosso grafo de exemplo
    int tamanho = 4;

    // 2. Define a Lista de Adjacência (NOVA FORMA)
    //    Isso cria um vetor com 4 posições, cada uma
    //    contendo um 'std::vector<Par>' vazio.
    Grafo grafo(tamanho);

    // 3. Adiciona as arestas (NOVA FORMA)
    //    Não preenchemos com INFINITO, apenas adicionamos as arestas
    //    que existem usando .push_back() ou .emplace_back().
    //    (Usando o grafo com índice 0-3)

    // De 0 para 1: Custo 5
    grafo[0].emplace_back(1, 5);
    // De 0 para 2: Custo 2
    grafo[0].emplace_back(2, 2);

    // De 1 para 3: Custo 6
    grafo[1].emplace_back(3, 6);

    // De 2 para 1: Custo 1
    grafo[2].emplace_back(1, 1);
    // De 2 para 3: Custo 10
    grafo[2].emplace_back(3, 10);

    // 4. Chama o algoritmo de Dijkstra (esta chamada não muda)
    auto resultado = dijkstra(grafo);

    // 5. Desempacota os resultados (do std::pair)
    std::vector<int> distanciasFinais = resultado.first;
    std::vector<int> predecessoresFinais = resultado.second;

    // 6. Imprime os resultados (Usando a correção de -1)
    std::cout << "Algoritmo de Dijkstra (Origem = 0)" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Vertice | Distancia | Predecessor" << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    for (int i = 0; i < tamanho; ++i) {
        std::cout << "   " << i << "    |     ";
        
        if (distanciasFinais[i] == INFINITO) {
            std::cout << "INF";
        } else {
            std::cout << distanciasFinais[i];
        }
        
        // (Assumindo que sua nova função usa -1 para "sem predecessor")
        std::cout << "     |      " << predecessoresFinais[i] << std::endl;
    }

    return 0;
}