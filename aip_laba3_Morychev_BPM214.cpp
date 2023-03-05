#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <set>

template<typename VertexType, typename WeightType>
class Graph {
private:
    std::vector<std::vector<WeightType>> matrix_;
    std::map<VertexType, int> vertex_map_;
    std::vector<VertexType> vertex_list_;

public:
    // Конструктор по умолчанию
    Graph() {}

    // Конструктор копирования
    Graph(const Graph& other) {
        matrix_ = other.matrix_;
        vertex_map_ = other.vertex_map_;
        vertex_list_ = other.vertex_list_;
    }

    // Конструктор перемещения
    Graph(Graph&& other) noexcept {
        matrix_ = std::move(other.matrix_);
        vertex_map_ = std::move(other.vertex_map_);
        vertex_list_ = std::move(other.vertex_list_);
    }

    // Оператор копирующего присваивания
    Graph& operator=(const Graph& other) {
        if (this != &other) {
            matrix_ = other.matrix_;
            vertex_map_ = other.vertex_map_;
            vertex_list_ = other.vertex_list_;
        }
        return *this;
    }

    // Оператор перемещающего присваивания
    Graph& operator=(Graph&& other) noexcept {
        if (this != &other) {
            matrix_ = std::move(other.matrix_);
            vertex_map_ = std::move(other.vertex_map_);
            vertex_list_ = std::move(other.vertex_list_);
        }
        return *this;
    }

    bool empty() const {
        return vertex_map_.empty();
    }

    size_t size() const {
        return vertex_map_.size();
    }

    void clear() {
        matrix_.clear();
        vertex_map_.clear();
        vertex_list_.clear();
    }

    void swap(Graph& other) noexcept {
        matrix_.swap(other.matrix_);
        vertex_map_.swap(other.vertex_map_);
        vertex_list_.swap(other.vertex_list_);
    }

    void add_vertex(VertexType vertex) {
        if (vertex_map_.find(vertex) == vertex_map_.end()) {
            int index = matrix_.size();
            matrix_.resize(index + 1);
            for (auto& row : matrix_) {
                row.resize(index + 1);
            }
            vertex_map_[vertex] = index;
            vertex_list_.push_back(vertex);
        }
    }

    void add_edge(VertexType from, VertexType to, WeightType weight) {
        int from_index = vertex_map_[from];
        int to_index = vertex_map_[to];
        matrix_[from_index][to_index] = weight;
    }

    // Получение веса ребра между вершинами from и to
    WeightType GetEdgeWeight(VertexType from, VertexType to) const {
        return matrix_[from][to];
    }

    // Получение всех исходящих ребер из вершины vertex
    std::list<std::pair<VertexType, WeightType>> GetOutgoingEdges(VertexType vertex) const {
        std::list<std::pair<VertexType, WeightType>> edges;
        for (auto it = matrix_[vertex].begin(); it != matrix_[vertex].end(); ++it) {
            if (it->second != WeightType()) {  // Игнорируем ребра с нулевым весом
                edges.emplace_back(it->first, it->second);
            }
        }
        return edges;
    }

    std::list<std::pair<VertexType, WeightType>> get_neighbors(VertexType vertex) const {
        std::list<std::pair<VertexType, WeightType>> neighbors;
        int vertex_index = vertex_map_.at(vertex);
        for (int i = 0; i < matrix_.size(); ++i) {
            if (matrix_[vertex_index][i] != 0) {
                neighbors.push_back(std::make_pair(vertex_list_[i], matrix_[vertex_index][i]));
            }
        }
        return neighbors;
    }

    std::set<VertexType> get_vertices() const {
        std::set<VertexType> vertices;
        for (const auto& vertex : vertex_list_) {
            vertices.insert(vertex);
        }
        return vertices;
    }

    using iterator = typename std::map<VertexType, int>::iterator;
    using const_iterator = typename std::map<VertexType, int>::const_iterator;

    iterator begin() { return vertex_map_.begin(); }
    iterator end() { return vertex_map_.end(); }
    const_iterator cbegin() const { return vertex_map_.cbegin(); }
    const_iterator cend() const { return vertex_map_.cend(); }


    size_t degree_in(VertexType vertex) const {
        size_t count = 0;
        int vertex_index = vertex_map_.at(vertex);
        for (int i = 0; i < matrix_.size(); ++i) {
            if (matrix_[i][vertex_index] != 0) {
                ++count;
            }
        }
        return count;
    }

    size_t degree_out(VertexType vertex) const {
        size_t count = 0;
        int vertex_index = vertex_map_.at(vertex);
        for (int i = 0; i < matrix_[vertex_index].size(); ++i) {
            if (matrix_[vertex_index][i] != 0) {
                ++count;
            }
        }
        return count;
    }

    bool loop(VertexType vertex) const {
        int vertex_index = vertex_map_.at(vertex);
        return matrix_[vertex_index][vertex_index] != 0;
    }

    void clear_edges() {
        for (auto& row : matrix_) {
            for (auto& weight : row) {
                weight = 0;
            }
        }
    }

    bool erase_edges_go_from(VertexType key) {
        if (vertex_map_.find(key) == vertex_map_.end()) {
            return false;
        }
        int index = vertex_map_[key];
        for (auto& weight : matrix_[index]) {
            weight = 0;
        }
        return true;
    }

    bool erase_edges_go_to(VertexType key) {
        if (vertex_map_.find(key) == vertex_map_.end()) {
            return false;
        }
        int index = vertex_map_[key];
        for (auto& row : matrix_) {
            row[index] = 0;
        }
        return true;
    }

    bool erase_node(VertexType key) {
        if (vertex_map_.find(key) == vertex_map_.end()) {
            return false;
        }
        int index = vertex_map_[key];
        vertex_map_.erase(key);
        vertex_list_.erase(vertex_list_.begin() + index);
        matrix_.erase(matrix_.begin() + index);
        for (auto& row : matrix_) {
            row.erase(row.begin() + index);
        }
        return true;
    }

};




