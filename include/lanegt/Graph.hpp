#pragma once

#include "LaneGTCore.hpp"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lanegt {

// Undirected simple graph with sorted CSR adjacency for exact fallback.
struct Graph {
    Vertex n = 0;
    std::vector<Edge> edges;  // canonical u < v
    std::vector<std::uint64_t> offsets;
    std::vector<Vertex> neighbors;

    static Graph from_edges(Vertex vertex_count, std::vector<Edge> input) {
        Graph graph;
        graph.n = vertex_count;
        for (Edge& e : input) {
            if (e.first >= vertex_count || e.second >= vertex_count)
                throw std::out_of_range("edge endpoint out of range");
            if (e.first > e.second) std::swap(e.first, e.second);
        }
        input.erase(std::remove_if(input.begin(), input.end(),
                       [](const Edge& e) { return e.first == e.second; }), input.end());
        std::sort(input.begin(), input.end());
        input.erase(std::unique(input.begin(), input.end()), input.end());
        graph.edges = std::move(input);
        graph.offsets.assign(static_cast<std::size_t>(graph.n) + 1, 0);
        for (auto [u, v] : graph.edges) {
            ++graph.offsets[u + 1];
            ++graph.offsets[v + 1];
        }
        std::partial_sum(graph.offsets.begin(), graph.offsets.end(), graph.offsets.begin());
        graph.neighbors.resize(graph.offsets.back());
        auto cursor = graph.offsets;
        for (auto [u, v] : graph.edges) {
            graph.neighbors[cursor[u]++] = v;
            graph.neighbors[cursor[v]++] = u;
        }
        for (Vertex u = 0; u < graph.n; ++u) {
            auto first = graph.neighbors.begin() + static_cast<std::ptrdiff_t>(graph.offsets[u]);
            auto last = graph.neighbors.begin() + static_cast<std::ptrdiff_t>(graph.offsets[u + 1]);
            std::sort(first, last);
        }
        return graph;
    }

    // Plain text: n m followed by m pairs of zero-based vertex IDs.
    static Graph read(const std::string& file) {
        std::ifstream in(file);
        if (!in) throw std::runtime_error("cannot open graph: " + file);
        std::uint64_t n, m;
        if (!(in >> n >> m) || n > std::numeric_limits<Vertex>::max())
            throw std::runtime_error("invalid graph header (expected: n m)");
        if (m > std::numeric_limits<std::size_t>::max())
            throw std::runtime_error("too many edges");
        std::vector<Edge> input;
        input.reserve(static_cast<std::size_t>(m));
        for (std::uint64_t i = 0; i < m; ++i) {
            std::uint64_t u, v;
            if (!(in >> u >> v) || u > std::numeric_limits<Vertex>::max() ||
                v > std::numeric_limits<Vertex>::max())
                throw std::runtime_error("invalid or truncated graph edge list");
            input.emplace_back(static_cast<Vertex>(u), static_cast<Vertex>(v));
        }
        std::string excess;
        if (in >> excess) throw std::runtime_error("extra data after m edges");
        return from_edges(static_cast<Vertex>(n), std::move(input));
    }

    bool has_edge(Vertex u, Vertex v) const {
        if (u >= n || v >= n) throw std::out_of_range("query vertex out of range");
        if (u == v) return false; // input is a simple graph
        return std::binary_search(neighbors.begin() + static_cast<std::ptrdiff_t>(offsets[u]),
                                  neighbors.begin() + static_cast<std::ptrdiff_t>(offsets[u + 1]), v);
    }
};

enum class QueryPath { Certificate, ExactFallback };
struct QueryResult { bool edge; QueryPath path; };

inline QueryResult edge_query(const Graph& graph, const LaneGT& index, Vertex u, Vertex v) {
    if (graph.n != index.vertex_count()) throw std::invalid_argument("graph / index mismatch");
    if (index.certifies_nonedge(u, v)) return {false, QueryPath::Certificate};
    return {graph.has_edge(u, v), QueryPath::ExactFallback};
}

} // namespace lanegt
