#include "lanegt/Graph.hpp"
#include "lanegt/Triangle.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

static lanegt::Vertex parse_vertex(const char* s) {
    const std::string token(s);
    if (token.empty() || token.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("expected nonnegative integer vertex ID");
    const unsigned long long x = std::stoull(token);
    if (x > std::numeric_limits<lanegt::Vertex>::max())
        throw std::out_of_range("vertex ID exceeds uint32");
    return static_cast<lanegt::Vertex>(x);
}

int main(int argc, char** argv) {
    try {
        std::string file;
        std::uint32_t p = 4;
        bool query = false, triangles = false, audit = false;
        lanegt::Vertex u = 0, v = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string opt = argv[i];
            if (opt == "--graph" && i + 1 < argc) file = argv[++i];
            else if (opt == "--lanes" && i + 1 < argc) p = parse_vertex(argv[++i]);
            else if (opt == "--query" && i + 2 < argc) {
                u = parse_vertex(argv[++i]); v = parse_vertex(argv[++i]); query = true;
            } else if (opt == "--triangles") triangles = true;
            else if (opt == "--audit") audit = true;
            else if (opt == "--help") {
                std::cout << "Usage: lanegt --graph FILE [--lanes 2|4|8]"
                             " [--query U V] [--triangles] [--audit]\n";
                return 0;
            } else throw std::invalid_argument("unknown or incomplete option: " + opt);
        }
        if (file.empty()) throw std::invalid_argument("provide --graph FILE (see --help)");
        const auto graph = lanegt::Graph::read(file);
        const auto t0 = std::chrono::steady_clock::now();
        auto index = lanegt::LaneGT::build(graph.n, graph.edges, p);
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << "vertices=" << graph.n << " edges=" << graph.edges.size()
                  << " lanes=" << index.lanes() << " bits_per_vertex=64"
                  << " bits_per_lane=" << index.bits_per_lane() << "\n";
        std::cout << "construction_seconds="
                  << std::chrono::duration<double>(t1 - t0).count() << "\n";
        if (query) {
            const auto result = lanegt::edge_query(graph, index, u, v);
            std::cout << "query " << u << " " << v << ": "
                      << (result.edge ? "EDGE" : "NONEDGE") << " ("
                      << (result.path == lanegt::QueryPath::Certificate ? "certificate" : "exact-fallback")
                      << ")\n";
        }
        if (triangles) std::cout << "triangles=" << lanegt::triangle_count(graph, index) << "\n";
        if (audit) std::cout << "edge_certificate_violations="
                             << index.count_safety_violations(graph.edges) << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
