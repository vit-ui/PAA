#pragma once
#include <vector>
#include <utility>
#include <random>
#include <climits>
#include <fstream>
#include <sstream>  // Para std::stringstream
#include <iomanip>  // Para std::setprecision e std::fixed
#include "../lib/json.hpp" // lib nlohmann/json

static const int INFINITO = INT_MAX;

// Função que gera um grafo para teste usando lista de adjacencia.
std::vector<std::vector<std::pair<int, int>>> geraGrafo(int tamanho, double densidade){
    // criando um gerador de numeros aleatórios usando static para persistir durante várias calls a função
    static std::random_device semente;
    static std::mt19937 motor(semente());
    std::uniform_int_distribution<int> escolheVertice(0, tamanho-1);
    std::uniform_int_distribution<int> escolhePeso(1, 100);

    std::vector<std::vector<std::pair<int, int>>> grafo(tamanho);
    
    // Calculando o número de arestas do grafo com base na densidade
    size_t maxArestas = densidade * tamanho * (tamanho-1);

    // Populando o grafo com pesos e direções aleatórias
    for(int i = 0; i < maxArestas; i++){
        int origem = escolheVertice(motor);
        int chegada = escolheVertice(motor);
        int peso = escolhePeso(motor);

        if(origem == chegada) continue;

        grafo[origem].emplace_back(chegada, peso);
    }
    return grafo;
}

// Função que coloca grafos em formato json em um arquivo txt
void salvaGrafo(int tamanho, double densidade, const std::vector<std::vector<std::pair<int, int>>>& grafo){
    // Preparando variáveis para a operação no arquivo   
    std::string caminhoGrafo = "../grafos/grafos.txt";
    nlohmann::json Grafos;

    // colocando os grafos existentes na memoria para atualizar o arquivo
    std::ifstream arquivoGrafosLeitura(caminhoGrafo);
    try{
        Grafos = nlohmann::json::parse(arquivoGrafosLeitura);
    } 
    catch(const std::exception& e){
        Grafos = nlohmann::json::array();
    }
    arquivoGrafosLeitura.close();

    // construindo o novo objeto json(grafo gerado)
    nlohmann::json novoGrafo;
    static int counter = 1;
    std::stringstream ss;
    ss << "Grafo_N" << tamanho << "D" << std::fixed << std::setprecision(2) << densidade << "_" << counter++;
    novoGrafo["id"] = ss.str();
    novoGrafo["parametros"]["tamanho"] = tamanho;
    novoGrafo["parametros"]["densidade"] = densidade;
    novoGrafo["grafo"] = grafo;
    novoGrafo["solucao"]["distancias"] = json::array(); // dt
    novoGrafo["solucao"]["predecessores"] = json::array(); // rot

    // atualizando lista de grafos
    Grafos.push_back(novoGrafo);

    // Colocando lista atualizada no arquivo
    std::ofstream arquivoGrafosEscrita(caminhoGrafo);
    arquivoGrafosEscrita << Grafos.dump(4);
}

// minDist e vizinhos não são mais utilizadas.
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