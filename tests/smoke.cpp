#include "lanegt/Graph.hpp"
#include "lanegt/Triangle.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

static std::uint64_t expected_triangles(const lanegt::Graph& g) {
    std::uint64_t count = 0;
    for (lanegt::Vertex u = 0; u < g.n; ++u)
        for (lanegt::Vertex v = u + 1; v < g.n; ++v)
            for (lanegt::Vertex w = v + 1; w < g.n; ++w)
                if (g.has_edge(u, v) && g.has_edge(v, w) && g.has_edge(u, w)) ++count;
    return count;
}

int main() {
    std::mt19937_64 rng(991);
    std::uint64_t graphs = 0, pair_checks = 0, triangle_checks = 0;
    for (lanegt::Vertex n = 0; n <= 15; ++n) {
        for (int seed = 0; seed < 36; ++seed) {
            std::vector<lanegt::Edge> edges;
            for (lanegt::Vertex u = 0; u < n; ++u)
                for (lanegt::Vertex v = u + 1; v < n; ++v)
                    if (rng() % 5 < static_cast<unsigned>(seed % 5)) edges.emplace_back(u,v);
            // Also exercise duplicate edges, reversed edges, and discarded self-loops.
            if (!edges.empty()) {
                edges.push_back(edges.front());
                edges.emplace_back(edges.front().second, edges.front().first);
            }
            if (n) edges.emplace_back(0,0);
            auto graph = lanegt::Graph::from_edges(n,edges);
            ++graphs;
            for (std::uint32_t p : {2U, 4U, 8U}) {
                auto idx = lanegt::LaneGT::build(n,edges,p);
                assert(idx.labels().size() == graph.n);
                assert(idx.count_safety_violations(graph.edges) == 0);
                for (lanegt::Vertex u = 0; u < n; ++u)
                    for (lanegt::Vertex v = 0; v < n; ++v) {
                        const auto r = lanegt::edge_query(graph,idx,u,v);
                        assert(r.edge == graph.has_edge(u,v));
                        if (r.path == lanegt::QueryPath::Certificate) assert(!r.edge);
                        ++pair_checks;
                    }
                assert(lanegt::triangle_count(graph,idx) == expected_triangles(graph));
                ++triangle_checks;
            }
        }
    }
    std::cout << "PASS graphs=" << graphs << " edge_queries=" << pair_checks
              << " triangle_checks=" << triangle_checks << "\n";
}
