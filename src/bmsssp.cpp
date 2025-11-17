#include <vector>
#include <set>
#include <queue>
#include <stack>
//#include <limits>

using Grafo = std::vector<std::vector<std::pair<int, int>>>;

// Placeholders
 //std::vector<double> distD(std::numeric_limits<double>::infinity());
 //distD[0] = 0.0;

std::pair<std::vector<int>, std::vector<int>> findPivots(const Grafo &grafo, std::vector<double> &distD, double limiteB, std::vector<int> fronteiraInicialS, int passosK)
{
    // Bellman-Ford
    std::vector<bool> verticesAlcancadosW(grafo.size(), false); // W
    std::vector<int> verticesAlcancadosWRetorno;
    std::vector<int> fronteiraAtualW_prev; // W_i-1. No caso: W_0

    for (int vertice : fronteiraInicialS) {
        if (!verticesAlcancadosW[vertice]) {
            verticesAlcancadosW[vertice] = true;
            verticesAlcancadosWRetorno.push_back(vertice);
            fronteiraAtualW_prev.push_back(vertice);
        }
    }

    std::vector<bool> adicionadoNestaCamada(grafo.size());
    for (int i = 0; i < passosK; i++) // no algo: i = 1 até k.
    {
        std::set<int> proximaFronteiraW_i;
        for (int verticeU : fronteiraAtualW_prev) // vertice u da camada anterior
        {
            for (const auto &aresta : grafo[verticeU]) // vizinho de u(v)
            {
                int verticeDestinoV = aresta.first; // v
                int peso = aresta.second; // peso u -> v
                double novoCusto = distD[verticeU] + peso; // distD[u] + peso[u,v]

                if (novoCusto <= distD[verticeDestinoV]) // novo menor caminho?
                {
                    distD[verticeDestinoV] = novoCusto; // Atualiza menor distancia

                    if (novoCusto < limiteB) // B é limite de distancia(janela que me importo)
                        proximaFronteiraW_i.insert(verticeDestinoV); // W_i U {v}
                }
            }
        }
        fronteiraAtualW_prev.clear();
        // faz W U W_i
        for (int vertice : proximaFronteiraW_i) {
            if (!verticesAlcancadosW[vertice]) {
                verticesAlcancadosW[vertice] = true;
                verticesAlcancadosWRetorno.push_back(vertice);
            }
            fronteiraAtualW_prev.push_back(vertice); // precisamos fazer a fronteira atual ir para a anterior. W_i-1 = W_i implicito no artigo.
        } 

        if (verticesAlcancadosWRetorno.size() > passosK * fronteiraInicialS.size())
            return std::make_pair(fronteiraInicialS, verticesAlcancadosWRetorno); // retorna S direto já que P = S aqui.
    }
    // Construção de P com F implicito
    std::vector<bool> pivots(grafo.size(), false);
    std::vector<int> pivotsRetorno;

    for (int vertice : fronteiraInicialS) {
        if (!verticesAlcancadosW[vertice]) continue;

        // Usando um DFS para percorrer F e achar os pivots.
        int contador = 0;
        std::set<int> visitados;
        std::stack<int> pilha;

        pilha.push(vertice);
        visitados.insert(vertice);

        while (!pilha.empty()) {
            int pai = pilha.top();
            pilha.pop();
            contador++;

            if (contador >= passosK) {
                if (!pivots[vertice]) {
                    pivots[vertice] = true;
                    pivotsRetorno.push_back(vertice);
                }
                break;
            }

            for (const auto& aresta : grafo[pai]) {
                int verticeDestino = aresta.first; // v
                int peso = aresta.second; // peso u -> v

                // Testes para garantir que percorremos somente vertices em F
                bool testes = verticesAlcancadosW[verticeDestino] && distD[verticeDestino] == distD[pai] + peso && !visitados.count(verticeDestino);
                if (testes) {
                    pilha.push(verticeDestino);
                    visitados.insert(verticeDestino);
                }
            }
        }
    }

    return std::make_pair(pivotsRetorno, verticesAlcancadosWRetorno);
}

std::pair<double, std::vector<int>> baseCase(const Grafo& grafo, std::vector<double>& distD, double limiteB, int pivoFonteS, int maxContagemK) {
    std::vector<int> verticesCompletosU_0; // a primeira iteração do while adiciona pivoFonteS
    // fila de prioridades H: (distancia, vertice)
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> filaFronteiraH;
    filaFronteiraH.push({ distD[pivoFonteS], pivoFonteS });

    while (!filaFronteiraH.empty() && verticesCompletosU_0.size() < maxContagemK + 1) {
        auto minPar = filaFronteiraH.top();
        filaFronteiraH.pop();

        int pesoVerticeAtual = minPar.first;
        int verticeAtualU = minPar.second;
        if (pesoVerticeAtual > distD[verticeAtualU]) continue; // isso cuida de duplicatas obsoletas na fila

        verticesCompletosU_0.push_back(verticeAtualU); // cada vertice é adicionado somente uma vez por causa da verificação acima

        for (auto& parVizinho : grafo[verticeAtualU]) {
            int vizinho = parVizinho.first; // v - verticeAtualU é u
            int pesoVizinho = parVizinho.second; // peso[u, v]

            int novoCusto = distD[verticeAtualU] + pesoVizinho;
            if (novoCusto <= distD[vizinho] && novoCusto < limiteB) {
                distD[vizinho] = novoCusto;

                // Não precisa verificar se já está na fila
                filaFronteiraH.push({ distD[vizinho], vizinho });
            }
        }
    }
    if (verticesCompletosU_0.size() <= maxContagemK) return std::make_pair(limiteB, verticesCompletosU_0);
    else { 
        // Encontrando a distancia maxima em U_0
        int Blinha = -1; // não tem distancia negativa pelo artigo
        for (int vertice : verticesCompletosU_0)
            if (Blinha < distD[vertice]) Blinha = distD[vertice];
        
        // Construindo U para retorno(retirando vertices onde distancia < Blinha
        std::vector<int> U;
        for (int vertice : verticesCompletosU_0)
            if (Blinha > distD[vertice]) U.push_back(vertice);

        return std::make_pair(Blinha, U);
    }
}