#pragma once

#include "Graph.hpp"
#include <array>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace lanegt {

// Degree-oriented forward graph. Each undirected edge is stored once.
// Adapted from the verified LaneGT selective-row Triangle implementation.
struct ForwardGraph {
    Vertex n = 0;
    std::vector<std::uint64_t> offsets;
    std::vector<Vertex> neighbors;

    static ForwardGraph build(const Graph& graph) {
        ForwardGraph f;
        f.n = graph.n;
        std::vector<std::uint64_t> deg(graph.n, 0);
        for (Vertex u = 0; u < graph.n; ++u)
            deg[u] = graph.offsets[u + 1] - graph.offsets[u];
        std::vector<Vertex> order(graph.n), rank(graph.n);
        std::iota(order.begin(), order.end(), Vertex{0});
        std::sort(order.begin(), order.end(), [&](Vertex u, Vertex v) {
            if (deg[u] != deg[v]) return deg[u] > deg[v];
            return u < v;
        });
        for (Vertex i = 0; i < graph.n; ++i) rank[order[i]] = i;
        f.offsets.assign(static_cast<std::size_t>(graph.n) + 1, 0);
        for (auto [u, v] : graph.edges)
            ++f.offsets[(rank[u] < rank[v] ? u : v) + 1];
        std::partial_sum(f.offsets.begin(), f.offsets.end(), f.offsets.begin());
        f.neighbors.resize(f.offsets.back());
        auto cursor = f.offsets;
        for (auto [u, v] : graph.edges) {
            const auto src = rank[u] < rank[v] ? u : v;
            const auto dst = rank[u] < rank[v] ? v : u;
            f.neighbors[cursor[src]++] = dst;
        }
        for (Vertex v = 0; v < f.n; ++v) {
            auto a = f.neighbors.begin() + static_cast<std::ptrdiff_t>(f.offsets[v]);
            auto b = f.neighbors.begin() + static_cast<std::ptrdiff_t>(f.offsets[v + 1]);
            std::sort(a, b);
        }
        return f;
    }
};

// Triangle certificates are aggregated by vertex-ID residue class. A blocked
// (r,s) class pair is sound when the entire relevant forward-neighbor classes
// share an independent-set bit in their responsible XOR lane.
struct TriangleMatrix {
    std::vector<std::uint64_t> blocked;
    std::vector<std::uint8_t> full, present, singleton;

    static TriangleMatrix build(const ForwardGraph& f, const LaneGT& index) {
        const std::uint32_t p = index.lanes();
        if (p != 2 && p != 4 && p != 8)
            throw std::invalid_argument("Triangle pruning currently supports 2, 4 or 8 lanes");
        if (f.n != index.vertex_count()) throw std::invalid_argument("graph / index mismatch");
        const std::uint32_t bits = 64 / p;
        const auto one_mask = (Word{1} << bits) - Word{1};
        const auto& labels = index.labels();
        TriangleMatrix mat;
        mat.blocked.assign(f.n, 0);
        mat.full.assign(f.n, 0);
        mat.present.assign(f.n, 0);
        mat.singleton.assign(f.n, 0);
        for (Vertex v = 0; v < f.n; ++v) {
            std::array<std::uint64_t, 8> count{};
            std::array<Word, 8> common{};
            common.fill(~Word{0});
            for (auto j = f.offsets[v]; j < f.offsets[v + 1]; ++j) {
                const Vertex u = f.neighbors[j];
                const auto r = u & (p - 1);
                ++count[r];
                common[r] &= labels[u];
            }
            std::uint32_t present = 0, singleton = 0;
            for (std::uint32_t r = 0; r < p; ++r) {
                if (count[r]) present |= (1U << r);
                if (count[r] == 1) singleton |= (1U << r);
            }
            mat.present[v] = static_cast<std::uint8_t>(present);
            mat.singleton[v] = static_cast<std::uint8_t>(singleton);
            if (f.offsets[v + 1] - f.offsets[v] < 2) continue;
            std::uint32_t active = 0, certified = 0;
            std::uint64_t blocked = 0;
            for (std::uint32_t r = 0; r < p; ++r) {
                for (std::uint32_t s = r; s < p; ++s) {
                    const bool has_pair = r == s ? count[r] >= 2 : (count[r] && count[s]);
                    if (!has_pair) continue;
                    ++active;
                    const std::uint32_t lane = r ^ s;
                    const Word mask = one_mask << (lane * bits);
                    if (common[r] & common[s] & mask) {
                        ++certified;
                        blocked |= (Word{1} << (r * p + s));
                        blocked |= (Word{1} << (s * p + r));
                    }
                }
            }
            mat.blocked[v] = blocked;
            if (active && active == certified) mat.full[v] = 1;
        }
        return mat;
    }
};

// Exactly counts undirected triangles, using LaneGT group certificates to
// skip impossible forward-neighbor pairs and exact membership for the rest.
inline std::uint64_t triangle_count(const Graph& graph, const LaneGT& index) {
    const auto forward = ForwardGraph::build(graph);
    const auto matrix = TriangleMatrix::build(forward, index);
    const std::uint32_t p = index.lanes();
    const std::uint32_t low = (1U << p) - 1U;
    std::vector<std::uint8_t> present(forward.n, 0);
    std::uint64_t triangles = 0;
    for (Vertex v = 0; v < forward.n; ++v) {
        if (forward.offsets[v + 1] - forward.offsets[v] < 2 || matrix.full[v]) continue;
        for (auto j = forward.offsets[v]; j < forward.offsets[v + 1]; ++j)
            present[forward.neighbors[j]] = 1;
        const std::uint64_t packed = matrix.blocked[v];
        const std::uint32_t class_mask = matrix.present[v];
        const std::uint32_t single = matrix.singleton[v];
        for (auto j = forward.offsets[v]; j < forward.offsets[v + 1]; ++j) {
            const Vertex u = forward.neighbors[j];
            const std::uint32_t r = u & (p - 1);
            const std::uint32_t blocked = (packed >> (r * p)) & low;
            std::uint32_t needed = class_mask;
            if (single & (1U << r)) needed &= ~(1U << r);
            if (needed && (blocked & needed) == needed) continue;
            if (!blocked) {
                for (auto k = forward.offsets[u]; k < forward.offsets[u + 1]; ++k)
                    triangles += present[forward.neighbors[k]];
            } else {
                for (auto k = forward.offsets[u]; k < forward.offsets[u + 1]; ++k) {
                    const Vertex w = forward.neighbors[k];
                    if (blocked & (1U << (w & (p - 1)))) continue;
                    triangles += present[w];
                }
            }
        }
        for (auto j = forward.offsets[v]; j < forward.offsets[v + 1]; ++j)
            present[forward.neighbors[j]] = 0;
    }
    return triangles;
}

} // namespace lanegt
