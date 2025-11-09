#include <vector>
#include <climits>
#include <algorithm>
#include <utility>
#include <iostream>

const int INFINITO = INT_MAX;

int minDist(const std::vector<int>& minDistancia, const std::vector<bool>& foiFechado){
    int indiceVerticeMinimo = -1;
    int distanciaMinimaAtual = INFINITO;

    for(int i = 0; i < minDistancia.size(); i++){
        if(!foiFechado[i] && minDistancia[i] < distanciaMinimaAtual){
            distanciaMinimaAtual = minDistancia[i];
            indiceVerticeMinimo = i;
        }
    }
    return indiceVerticeMinimo;
}


std::vector<int> vizinhos(const std::vector<std::vector<int>>& grafo, const std::vector<bool>& foiFechado, int verticeAtual){
    std::vector<int> vizinhos;
    for(int i = 0; i < grafo.size();i++){
        if(!foiFechado[i] && grafo[verticeAtual][i] != INFINITO)
            vizinhos.emplace_back(i);
    }
    return vizinhos;
}

std::pair<std::vector<int>, std::vector<int>> dijkstra(const std::vector<std::vector<int>>& grafo){
    int tamanho = grafo.size();

    std::vector<int> minDistancia(tamanho, INFINITO); //dt
    std::vector<int> predecessores(tamanho, -1); //rot
    std::vector<bool> foiFechado(tamanho, false); // fechado -> true | aberto -> false

    minDistancia[0] = 0;

    int numFechados = 0;
    while(numFechados != tamanho){
        int verticeAtual = minDist(minDistancia, foiFechado); // r
        foiFechado[verticeAtual] = true;
        numFechados++;
        std::vector<int> vizinhosAbertos = vizinhos(grafo, foiFechado, verticeAtual);

        for(const auto& vizinho : vizinhosAbertos){
            // linha 14 era redundante. Pulei direto para 15.
            int distanciaNova = minDistancia[verticeAtual] + grafo[verticeAtual][vizinho];
            if(distanciaNova < minDistancia[vizinho]){
                minDistancia[vizinho] = distanciaNova;
                predecessores[vizinho] = verticeAtual;
            }
        }
    }
    return std::make_pair(minDistancia, predecessores);
}

int main() {
    
    // 1. Define o tamanho do nosso grafo de exemplo
    int tamanho = 4;

    // 2. Define a Matriz de Adjacência (nosso 'grafo')
    // Começa tudo com INFINITO
    std::vector<std::vector<int>> grafo(tamanho, std::vector<int>(tamanho, INFINITO));

    // Define o custo de ir para si mesmo como 0
    for (int i = 0; i < tamanho; ++i) {
        grafo[i][i] = 0;
    }

    // 3. Adiciona as arestas do nosso exemplo:
    // (Vértices 0, 1, 2, 3)
    grafo[0][1] = 5;  // De 0 para 1: Custo 5
    grafo[0][2] = 2;  // De 0 para 2: Custo 2
    grafo[1][3] = 6;  // De 1 para 3: Custo 6
    grafo[2][1] = 1;  // De 2 para 1: Custo 1
    grafo[2][3] = 10; // De 2 para 3: Custo 10

    // 4. Chama o algoritmo de Dijkstra
    auto resultado = dijkstra(grafo);

    // 5. Desempacota os resultados (do std::pair)
    std::vector<int> distanciasFinais = resultado.first;
    std::vector<int> predecessoresFinais = resultado.second;

    // 6. Imprime os resultados
    std::cout << "Algoritmo de Dijkstra (Origem = 0)" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Vertice | Distancia | Predecessor" << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    for (int i = 0; i < tamanho; ++i) {
        std::cout << "   " << i + 1 << "    |     ";
        
        if (distanciasFinais[i] == INFINITO) {
            std::cout << "INF";
        } else {
            std::cout << distanciasFinais[i];
        }
        
        std::cout << "     |      " << predecessoresFinais[i] << std::endl;
    }

    return 0;
}