#ifndef ROAD_GRAPH_LIBRARY_H
#define ROAD_GRAPH_LIBRARY_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>

/**
 * @file road_graph_library.h
 * @brief Заголовочный файл, содержащий определение классов и структур для работы с дорожным графом.
 */

/**
 * @struct RoadSegment
 * @brief Represents a segment of the road network.
 *
 * Данная структура описывает один участок дороги. 
 * Его поля соответствуют бинарному формату:
 * - 8 байт (uint64_t) для `start` (ID начала)
 * - 8 байт (uint64_t) для `end` (ID конца)
 * - 8 байт (double) для `length` (длина в метрах)
 * - 8 байт (double) для `speed` (скорость в км/ч)
 */
struct RoadSegment {
    uint64_t start; ///< Start node ID (идентификатор начала участка)
    uint64_t end;   ///< End node ID (идентификатор конца участка)
    double length;  ///< Length of the segment in meters (длина участка)
    double speed;   ///< Average speed in km/h (средняя скорость на участке)
};

/**
 * @struct NodeCoordinates
 * @brief Represents coordinates of a node in the road network.
 *
 * Данная структура описывает координаты узла (вершины) графа.
 * Поля соответствуют формату бинарного файла:
 * - 8 байт (uint64_t) для `id`
 * - 8 байт (double) для `x`
 * - 8 байт (double) для `y`
 */
struct NodeCoordinates {
    uint64_t id; ///< Node ID (идентификатор вершины)
    double x;    ///< X coordinate (e.g. longitude)
    double y;    ///< Y coordinate (e.g. latitude)
};

/**
 * @class RoadGraph
 * @brief Represents a road graph with methods to process road network data.
 *
 * Класс, содержащий структуры данных для хранения дорожного графа и методы для:
 * - Загрузки бинарных файлов с данными об участках дорог (`loadGraphData`)
 *   и координатах узлов (`loadNodeData`).
 * - Проверки достижимости одной вершины из другой (`isReachable`).
 * - Поиска кратчайшего пути (`findShortestPath`) и самого быстрого пути (`findFastestPath`).
 * - Получения длины и скорости конкретного ребра (`getEdgeLength` и `getEdgeSpeed`).
 * - Получения списка всех пар ребер (`getEdgePairs`).
 */
class RoadGraph {
public:
    /**
     * @brief Loads road segments from a binary file.
     *
     * Функция считывает из файла структуры RoadSegment. В случае обнаружения
     * некорректных данных (длина <= 0 или скорость <= 0) выводит warning и пропускает запись.
     * 
     * @param filePath Путь к бинарному файлу с данными.
     * @throws std::runtime_error Если не удалось открыть файл.
     */
    void loadGraphData(const std::string& filePath);

    /**
     * @brief Loads node coordinates from a binary file.
     *
     * Считывает из файла структуры NodeCoordinates и заполняет 
     * таблицу координат.
     *
     * @param filePath Путь к бинарному файлу с координатами.
     * @throws std::runtime_error Если не удалось открыть файл.
     */
    void loadNodeData(const std::string& filePath);

    /**
     * @brief Checks reachability of one node from another.
     *
     * Выполняет обход (BFS/DFS) по списку смежности, чтобы определить,
     * можно ли дойти от `start` до `end`.
     *
     * @param start ID стартовой вершины.
     * @param end ID конечной вершины.
     * @return true, если достижимо; false в противном случае.
     */
    bool isReachable(uint64_t start, uint64_t end);

    /**
     * @brief Finds the shortest path by distance.
     *
     * Реализует алгоритм Дейкстры, используя в качестве веса длину ребра.
     *
     * @param start ID стартовой вершины.
     * @param end ID конечной вершины.
     * @return Вектор с последовательностью вершин от start до end (включая обе).
     *         Пустой вектор, если путь не найден.
     */
    std::vector<uint64_t> findShortestPath(uint64_t start, uint64_t end);

    /**
     * @brief Finds the fastest path by time.
     *
     * Реализует алгоритм Дейкстры, используя в качестве веса время (секунды) прохождения ребра.
     *
     * @param start ID стартовой вершины.
     * @param end ID конечной вершины.
     * @return Вектор с последовательностью вершин от start до end (включая обе).
     *         Пустой вектор, если путь не найден.
     */
    std::vector<uint64_t> findFastestPath(uint64_t start, uint64_t end);

    /**
     * @brief Returns the length of an edge in meters.
     *
     * Находит ребро `(start -> end)` в `adjacencyList` и возвращает его длину (в метрах).
     *
     * @param start ID вершины-источника.
     * @param end ID вершины-приёмника.
     * @return Длина в метрах или `std::numeric_limits<double>::infinity()`, если нет такого ребра.
     */
    double getEdgeLength(uint64_t start, uint64_t end);

    /**
     * @brief Returns the speed on an edge.
     *
     * Возвращает скорость в м/с (или в км/ч — в зависимости от реализации) 
     * на ребре `(start -> end)`, вычисляя её по данным о длине и времени из `timeWeightedGraph`.
     *
     * @param start ID вершины-источника.
     * @param end ID вершины-приёмника.
     * @return Скорость на участке (м/с или км/ч) либо 
     *         `std::numeric_limits<double>::infinity()`, если ребра нет.
     */
    double getEdgeSpeed(uint64_t start, uint64_t end);

    /**
     * @brief Returns all edge pairs from the adjacency list.
     *
     * Формирует вектор пар (start, end) для всех рёбер в графе.
     * Используется, например, для массовой обработки.
     *
     * @return Вектор пар (start, end).
     */
    std::vector<std::pair<uint64_t, uint64_t>> getEdgePairs() const;

private:
    /**
     * @brief Adjacency list for distance-based graph.
     *
     * Ключ — это ID вершины, значение — вектор пар (конечная_вершина, длина_в_метрах).
     */
    std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, double>>> adjacencyList;

    /**
     * @brief Adjacency list for time-based graph.
     *
     * Ключ — это ID вершины, значение — вектор пар (конечная_вершина, время_в_секундах).
     */
    std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, double>>> timeWeightedGraph;

    /**
     * @brief Map of node coordinates.
     *
     * Сопоставляет ID вершины с её координатами (x, y).
     */
    std::unordered_map<uint64_t, NodeCoordinates> nodeCoordinates;
};

#endif // ROAD_GRAPH_LIBRARY_H
