# RoadGraph

Проект **RoadGraph** представляет собой библиотеку и консольную утилиту для анализа дорожного графа.  

## Возможности

1. **Загрузка данных** о дорогах (участках) и узлах (координатах) из бинарных файлов:
   - Поддержка чтения `RoadSegment` (id начала, id конца, длина в метрах, скорость в км/ч).
   - Поддержка чтения `NodeCoordinates` (id узла, координаты x, y).
2. **Построение внутреннего представления** графа в виде:
   - `adjacencyList` — хранение рёбер по длине (метры).
   - `timeWeightedGraph` — хранение рёбер по времени (секунды).
3. **Функциональность**:
   - Проверка достижимости: `isReachable(start, end)`.
   - Поиск кратчайшего пути: `findShortestPath(start, end)` (алгоритм Дейкстры по длине).
   - Поиск самого быстрого пути: `findFastestPath(start, end)` (алгоритм Дейкстры по времени).
   - Получение длины конкретного ребра (`getEdgeLength`) и вычисление скорости (`getEdgeSpeed`).
4. **Утилита командной строки** `road_graph_utility`, позволяющая:
   - Проверять достижимость между двумя узлами.
   - Находить кратчайший/самый быстрый путь.
   - При режиме `auto` — обрабатывать все доступные пары рёбер, извлечённые из графа.

---

## Сборка и установка

Проект использует [CMake](https://cmake.org/). Минимально поддерживаемая версия — 3.10.

1. **Клонируйте** репозиторий (или получите исходные файлы любым другим удобным способом).
2. **Перейдите** в корень проекта и создайте папку сборки:
```bash
   mkdir build
   cd build
   cmake ..
```
   На данном этапе проект собран и готов к использованию.
   В директории build сгенерирован исполняемый файл утилиты `road_graph_utility` и файл библиотеки `libroadgraph.so`.

3. **Установка библиотеки** (опционально)
```bash
   sudo cmake --build . --target install
```
   По умолчанию файлы будут установлены в системные директории, которые определяет `CMAKE_INSTALL_PREFIX`.
   Если хотите установить их в другую директорию (например, ~/myroadgraph), то при конфигурации задайте:
```bash
   cmake -DCMAKE_INSTALL_PREFIX=~/myroadgraph ..
   cmake --build . --target install
```
4. **Удаление библиотеки**
```bash 
   sudo cmake --build . --target remove
```
## Использование утилиты
После сборки (и установки) вы можете запустить утилиту: 
```bash
   ./road_graph_utility <graph_data> <node_data> <mode> [start_node end_node] 
```
-  `graph_data`: путь к бинарному файлу с данными об участках дорог (RoadSegment).
-  `node_data`: путь к бинарному файлу с координатами узлов (NodeCoordinates).
-  `mode`: один из вариантов:
    -  reachable — проверить достижимость (требуются start_node и end_node).
    -  shortest — найти кратчайший путь (требуются start_node и end_node).
    -  fastest — найти самый быстрый путь (требуются start_node и end_node).
    -  auto — обойти все пары рёбер, извлечённые из графа, и вывести для них кратчайший/быстрый путь (start_node и end_node не нужны).
   start_node и end_node (указываются только при соответствующих режимах).

Например: 
```bash
   ./road_graph_utility manhattan_edges.bin manhattan_nodes.bin shortest 30807307 30807310 
```

На датасете представленном в папке `data` ожидается вывод:
```bash 
   Processing pair: 30807307 -> 30807310
   Shortest Path: 30807307 -> 4276626653 -> 4276626638 -> 4276626627 -> 4276626608 -> 4276626593 -> 30807309 -> 30807310
   Total Distance: 39.4663 meters 
```

## Контактные данные

**Мищенко Игорь Андреевич**  
[igor.mishchenko.work@gmail.com](mailto:igor.mishchenko.work@gmail.com)
