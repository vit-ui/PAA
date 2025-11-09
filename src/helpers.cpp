#include <vector>
#include <utility>
#include <random>

// Função que gera um grafo para teste usando lista de adjacencia.
std::vector<std::vector<std::pair<int, int>>> geraGrafo(int tamanho, double densidade){
    // criando um gerador de numeros aleatórios usando static para persistir durante varias calls a função
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