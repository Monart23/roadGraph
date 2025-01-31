#include "road_graph_library.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>
#include <algorithm>

/**
 * @file road_graph_utility.cpp
 * @brief Пример консольной утилиты для работы с классом RoadGraph.
 */

/**
 * @brief Выводит справочную информацию о режиме использования.
 */
void printHelp() {
    std::cout << "Usage: road_graph_utility <graph_data> <node_data> <mode> [start_node end_node]" << std::endl;
    std::cout << "\nModes:" << std::endl;
    std::cout << "  reachable - Check if a node is reachable" << std::endl;
    std::cout << "  shortest  - Find the shortest path" << std::endl;
    std::cout << "  fastest   - Find the fastest path" << std::endl;
    std::cout << "  auto      - Process all node pairs automatically" << std::endl;
    std::cout << "\nIf start_node and end_node are provided, only that pair is processed." << std::endl;
}

/**
 * @brief Точка входа (main) для консольной утилиты.
 *
 * @param argc Количество аргументов командной строки.
 * @param argv Массив аргументов.
 * @return Код возврата (0 при успехе, 1 при ошибке).
 */
int main(int argc, char* argv[]) {
    if (argc < 4) {
        printHelp();
        return 1;
    }
    
    std::string graphFile = argv[1];
    std::string nodeFile = argv[2];
    std::string mode = argv[3];
    bool singleQuery = (argc == 6);
    uint64_t startNode = 0, endNode = 0;
    
    if (singleQuery) {
        startNode = std::stoull(argv[4]);
        endNode = std::stoull(argv[5]);
    }
    
    RoadGraph graph;
    try {
        graph.loadGraphData(graphFile);
        graph.loadNodeData(nodeFile);
    } catch (const std::exception& e) {
        std::cerr << "Error loading data: " << e.what() << std::endl;
        return 1;
    }
    
    // Формируем список пар вершин
    std::vector<std::pair<uint64_t, uint64_t>> nodePairs;
    if (singleQuery) {
        nodePairs.emplace_back(startNode, endNode);
    } else {
        nodePairs = graph.getEdgePairs();
    }
    
    // Обработка каждой пары
    for (const auto& [start, end] : nodePairs) {
        std::cout << "Processing pair: " << start << " -> " << end << std::endl;
        
        if (mode == "reachable") {
            std::cout << "Reachable: " 
                      << (graph.isReachable(start, end) ? "YES" : "NO") 
                      << std::endl;
        } 
        else if (mode == "shortest" || mode == "auto") {
            auto shortestPath = graph.findShortestPath(start, end);
            double totalLength = 0.0;
            std::cout << "Shortest Path: ";
            for (size_t i = 0; i < shortestPath.size(); ++i) {
                std::cout << shortestPath[i];
                if (i < shortestPath.size() - 1) {
                    std::cout << " -> ";
                }
                // Суммируем длину
                if (i > 0) {
                    totalLength += graph.getEdgeLength(shortestPath[i - 1], shortestPath[i]);
                }
            }
            std::cout << "\nTotal Distance: " << totalLength << " meters" << std::endl;
        }
        
        if (mode == "fastest" || mode == "auto") {
            auto fastestPath = graph.findFastestPath(start, end);
            double totalTime_s = 0.0;
            std::cout << "Fastest Path: ";
            for (size_t i = 0; i < fastestPath.size(); ++i) {
                std::cout << fastestPath[i];
                if (i < fastestPath.size() - 1) {
                    std::cout << " -> ";
                }
                // Считаем общее время, если есть ребро
                if (i > 0) {
                    double length = graph.getEdgeLength(fastestPath[i - 1], fastestPath[i]); 
                    double speed_m_s = graph.getEdgeSpeed(fastestPath[i - 1], fastestPath[i]);
                    if (!std::isinf(speed_m_s) && speed_m_s > 0.0) {
                        double edgeTime = length / speed_m_s; // метры / (м/с) = секунды
                        totalTime_s += edgeTime;
                    } else {
                        // Нет скорости / бесконечность => ребро некорректно
                    }
                }
            }
            std::cout << "\nTotal Time: " << totalTime_s << " seconds" << std::endl;
        }
    }
    return 0;
}
