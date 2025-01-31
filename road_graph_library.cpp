#include "road_graph_library.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>
#include <algorithm>

void RoadGraph::loadGraphData(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open graph data file.");
    }
    
    while (!file.eof()) {
        RoadSegment segment;
        file.read(reinterpret_cast<char*>(&segment), sizeof(RoadSegment));
        if (file.gcount() == sizeof(RoadSegment)) {
            adjacencyList[segment.start].emplace_back(segment.end, segment.length);
            timeWeightedGraph[segment.start].emplace_back(segment.end, segment.length / (segment.speed / 3.6));
        }
    }
    file.close();
}

void RoadGraph::loadNodeData(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open node data file.");
    }
    
    while (!file.eof()) {
        NodeCoordinates node;
        file.read(reinterpret_cast<char*>(&node), sizeof(NodeCoordinates));
        if (file.gcount() == sizeof(NodeCoordinates)) {
            nodeCoordinates[node.id] = node;
        }
    }
    file.close();
}

bool RoadGraph::isReachable(uint64_t start, uint64_t end) {
    if (adjacencyList.find(start) == adjacencyList.end()) return false;
    
    std::queue<uint64_t> q;
    std::unordered_map<uint64_t, bool> visited;
    
    q.push(start);
    visited[start] = true;
    
    while (!q.empty()) {
        uint64_t node = q.front();
        q.pop();
        
        if (node == end) return true;
        
        for (const auto& neighbor : adjacencyList[node]) {
            if (!visited[neighbor.first]) {
                visited[neighbor.first] = true;
                q.push(neighbor.first);
            }
        }
    }
    return false;
}

std::vector<std::pair<uint64_t, uint64_t>> RoadGraph::getEdgePairs() const {
    std::vector<std::pair<uint64_t, uint64_t>> edges;
    for (const auto& [start, adjList] : adjacencyList) {
        for (const auto& [end, _] : adjList) {
            edges.emplace_back(start, end);
        }
    }
    return edges;
}

double RoadGraph::getEdgeLength(uint64_t start, uint64_t end) {
    if (adjacencyList.find(start) != adjacencyList.end()) {
        for (const auto& neighbor : adjacencyList[start]) {
            if (neighbor.first == end) {
                return neighbor.second;
            }
        }
    }
    return std::numeric_limits<double>::infinity();
}

double RoadGraph::getEdgeSpeed(uint64_t start, uint64_t end) {
    // Найдём длину (метры)
    double length_m = getEdgeLength(start, end);

    // Найдём время (секунды)
    if (timeWeightedGraph.find(start) != timeWeightedGraph.end()) {
        for (const auto& neighbor : timeWeightedGraph[start]) {
            if (neighbor.first == end) {
                double time_s = neighbor.second; 
                if (time_s > 0.0) {
                    double speed_m_s = length_m / time_s; 
                    double speed_km_h = speed_m_s * 3.6;
                    return speed_km_h;
                }
            }
        }
    }
    return std::numeric_limits<double>::infinity();
}


std::vector<uint64_t> RoadGraph::findShortestPath(uint64_t start, uint64_t end) {
    std::unordered_map<uint64_t, double> distances;
    std::unordered_map<uint64_t, uint64_t> predecessors;
    std::priority_queue<std::pair<double, uint64_t>, std::vector<std::pair<double, uint64_t>>, std::greater<>> pq;

    for (const auto& pair : adjacencyList) {
        distances[pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [current_dist, node] = pq.top();
        pq.pop();

        if (node == end) break;

        for (const auto& neighbor : adjacencyList[node]) {
            double new_dist = current_dist + neighbor.second;
            if (new_dist < distances[neighbor.first]) {
                distances[neighbor.first] = new_dist;
                predecessors[neighbor.first] = node;
                pq.emplace(new_dist, neighbor.first);
            }
        }
    }

    if (predecessors.find(end) == predecessors.end()) {
        return {};  // Нет пути
    }

    std::vector<uint64_t> path;
    for (uint64_t at = end; at != start; at = predecessors[at]) {
        if (predecessors.find(at) == predecessors.end()) {
            return {};  // Ошибка при восстановлении пути
        }
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}


std::vector<uint64_t> RoadGraph::findFastestPath(uint64_t start, uint64_t end) {
    std::unordered_map<uint64_t, double> times;
    std::unordered_map<uint64_t, uint64_t> predecessors;
    std::priority_queue<std::pair<double, uint64_t>, std::vector<std::pair<double, uint64_t>>, std::greater<>> pq;

    for (const auto& pair : timeWeightedGraph) {
        times[pair.first] = std::numeric_limits<double>::infinity();
    }
    times[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [current_time, node] = pq.top();
        pq.pop();

        if (node == end) break;

        for (const auto& neighbor : timeWeightedGraph[node]) {
            double new_time = current_time + neighbor.second;
            if (new_time < times[neighbor.first]) {
                times[neighbor.first] = new_time;
                predecessors[neighbor.first] = node;
                pq.emplace(new_time, neighbor.first);
            }
        }
    }

    if (predecessors.find(end) == predecessors.end()) {
        return {};  // Нет пути
    }

    std::vector<uint64_t> path;
    for (uint64_t at = end; at != start; at = predecessors[at]) {
        if (predecessors.find(at) == predecessors.end()) {
            return {};  // Ошибка при восстановлении пути
        }
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}
