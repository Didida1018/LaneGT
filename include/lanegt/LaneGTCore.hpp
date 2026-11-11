#pragma once

// LaneGTCore.hpp
// Standalone LaneGT implementation: no legacy GT, no benchmark, no file parser.
//
// Core guarantees:
//   1. Each undirected edge (u,v) is routed to exactly one lane:
//          lane(u,v) = (u XOR v) & (P - 1)
//      where P is a power of two and divides 64.
//   2. Each lane owns 64/P bits in one 64-bit label per vertex.
//   3. Every bit is a maximal independent set (MIS) of that lane subgraph.
//   4. If two vertices share a bit in their routed lane, they are guaranteed
//      to be a non-edge in the original graph.
//
// Typical query integration:
//   if (index.certifies_nonedge(u, v)) return false; // exact non-edge certificate
//   return exact_graph.has_edge(u, v);               // exact fallback
//
// Input contract:
//   - Vertex IDs are in [0, vertex_count).
//   - Edges are interpreted as undirected.
//   - build() removes self-loops and duplicate undirected edges.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lanegt {

using Vertex = std::uint32_t;
using Word = std::uint64_t;
using Edge = std::pair<Vertex, Vertex>;

class SplitMix64 {
public:
    using result_type = std::uint64_t;

    explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

    static constexpr result_type min() { return 0; }
    static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }

    result_type operator()() {
        std::uint64_t z = (state_ += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30U)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27U)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31U);
    }

private:
    std::uint64_t state_;
};

inline std::uint64_t mix64(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
}

class LaneGT {
public:
    LaneGT() = default;

    // Build a LaneGT certificate index.
    // lanes must be one of 1,2,4,8,16,32,64.
    static LaneGT build(std::uint32_t vertex_count,
                        std::vector<Edge> edges,
                        std::uint32_t lanes,
                        std::uint64_t bucket_seed = 20260708ULL) {
        validate_lane_count(lanes);
        normalize_edges(vertex_count, edges);

        LaneGT index;
        index.vertex_count_ = vertex_count;
        index.lanes_ = lanes;
        index.bits_per_lane_ = 64U / lanes;
        index.bucket_seed_ = bucket_seed;
        index.labels_.assign(vertex_count, Word{0});
        index.lane_edge_counts_.assign(lanes, 0);
        index.initialize_lane_masks();

        // Route every real edge to exactly one lane.
        std::vector<std::vector<Edge>> lane_edges(lanes);
        const std::size_t reserve_each = edges.size() / lanes + 1;
        for (auto& bucket : lane_edges) bucket.reserve(reserve_each);

        for (const Edge& edge : edges) {
            const std::uint32_t lane = route(edge.first, edge.second, lanes);
            lane_edges[lane].push_back(edge);
        }

        // Build 64/P bucket-shuffled MIS bits independently in each lane.
        // Only one lane CSR is materialized at a time.
        for (std::uint32_t lane = 0; lane < lanes; ++lane) {
            index.lane_edge_counts_[lane] = lane_edges[lane].size();
            CSRGraph local = make_csr(vertex_count, std::move(lane_edges[lane]));

            const std::uint64_t lane_seed = mix64(
                bucket_seed
                ^ (static_cast<std::uint64_t>(lanes) << 48U)
                ^ (static_cast<std::uint64_t>(lane) * 0x9e3779b97f4a7c15ULL));

            append_bucket_mis_bits(
                local,
                lane * index.bits_per_lane_,
                index.bits_per_lane_,
                lane_seed,
                index.labels_);
        }

        return index;
    }

    std::uint32_t vertex_count() const noexcept { return vertex_count_; }
    std::uint32_t lanes() const noexcept { return lanes_; }
    std::uint32_t bits_per_lane() const noexcept { return bits_per_lane_; }
    std::uint64_t bucket_seed() const noexcept { return bucket_seed_; }

    const std::vector<Word>& labels() const noexcept { return labels_; }
    const std::vector<std::uint64_t>& lane_edge_counts() const noexcept {
        return lane_edge_counts_;
    }

    // O(1) deterministic pair routing.
    std::uint32_t route_lane(Vertex u, Vertex v) const noexcept {
        return route(u, v, lanes_);
    }

    // Returns the shared certificate bits in the unique routed lane.
    Word routed_common_bits(Vertex u, Vertex v) const {
        check_vertex(u);
        check_vertex(v);
        const Word common = labels_[u] & labels_[v];
        if (lanes_ == 1U) return common;
        return common & lane_masks_[route_lane(u, v)];
    }

    // True means (u,v) is provably a non-edge and exact adjacency lookup may
    // be skipped. False means "unknown" and requires exact fallback.
    bool certifies_nonedge(Vertex u, Vertex v) const {
        return routed_common_bits(u, v) != Word{0};
    }

    // Optional correctness audit. Returns the number of real edges that would
    // be incorrectly pruned; a correct build must return zero.
    std::uint64_t count_safety_violations(const std::vector<Edge>& real_edges) const {
        std::uint64_t violations = 0;
        for (const Edge& e : real_edges) {
            if (e.first >= vertex_count_ || e.second >= vertex_count_) {
                throw std::out_of_range("edge endpoint outside LaneGT vertex range");
            }
            violations += certifies_nonedge(e.first, e.second) ? 1ULL : 0ULL;
        }
        return violations;
    }

private:
    struct CSRGraph {
        std::uint32_t n = 0;
        std::vector<std::uint64_t> offsets;
        std::vector<Vertex> neighbors;
    };

    std::uint32_t vertex_count_ = 0;
    std::uint32_t lanes_ = 1;
    std::uint32_t bits_per_lane_ = 64;
    std::uint64_t bucket_seed_ = 0;
    std::vector<Word> labels_;
    std::array<Word, 64> lane_masks_{};
    std::vector<std::uint64_t> lane_edge_counts_;

    static void validate_lane_count(std::uint32_t lanes) {
        if (lanes == 0U || lanes > 64U || (lanes & (lanes - 1U)) != 0U ||
            (64U % lanes) != 0U) {
            throw std::invalid_argument(
                "LaneGT lane count must be a power of two dividing 64");
        }
    }

    static std::uint32_t route(Vertex u, Vertex v, std::uint32_t lanes) noexcept {
        return lanes == 1U ? 0U : ((u ^ v) & (lanes - 1U));
    }

    void initialize_lane_masks() {
        lane_masks_.fill(Word{0});
        if (lanes_ == 1U) {
            lane_masks_[0] = ~Word{0};
            return;
        }

        const Word local_mask = (Word{1} << bits_per_lane_) - Word{1};
        for (std::uint32_t lane = 0; lane < lanes_; ++lane) {
            lane_masks_[lane] = local_mask << (lane * bits_per_lane_);
        }
    }

    void check_vertex(Vertex v) const {
        if (v >= vertex_count_) {
            throw std::out_of_range("LaneGT vertex ID out of range");
        }
    }

    static void normalize_edges(std::uint32_t n, std::vector<Edge>& edges) {
        std::vector<Edge> normalized;
        normalized.reserve(edges.size());

        for (Edge e : edges) {
            if (e.first >= n || e.second >= n) {
                throw std::out_of_range("edge endpoint outside vertex range");
            }
            if (e.first == e.second) continue;
            if (e.first > e.second) std::swap(e.first, e.second);
            normalized.push_back(e);
        }

        std::sort(normalized.begin(), normalized.end());
        normalized.erase(std::unique(normalized.begin(), normalized.end()), normalized.end());
        edges.swap(normalized);
    }

    static CSRGraph make_csr(std::uint32_t n, std::vector<Edge> edges) {
        CSRGraph graph;
        graph.n = n;
        graph.offsets.assign(static_cast<std::size_t>(n) + 1U, 0);

        for (const Edge& e : edges) {
            ++graph.offsets[e.first + 1U];
            ++graph.offsets[e.second + 1U];
        }
        std::partial_sum(graph.offsets.begin(), graph.offsets.end(), graph.offsets.begin());

        graph.neighbors.resize(graph.offsets.back());
        std::vector<std::uint64_t> cursor = graph.offsets;
        for (const Edge& e : edges) {
            graph.neighbors[cursor[e.first]++] = e.second;
            graph.neighbors[cursor[e.second]++] = e.first;
        }
        return graph;
    }

    static void greedy_mis(const CSRGraph& graph,
                           const std::vector<Vertex>& order,
                           std::vector<std::int8_t>& state) {
        // -1: undecided, 0: excluded by a selected neighbor, 1: selected.
        std::fill(state.begin(), state.end(), std::int8_t{-1});

        for (Vertex v : order) {
            if (state[v] != -1) continue;
            state[v] = 1;

            for (std::uint64_t i = graph.offsets[v]; i < graph.offsets[v + 1U]; ++i) {
                const Vertex neighbor = graph.neighbors[i];
                if (state[neighbor] == -1) state[neighbor] = 0;
            }
        }
    }

    static void append_bucket_mis_bits(const CSRGraph& graph,
                                       std::uint32_t bit_offset,
                                       std::uint32_t bit_count,
                                       std::uint64_t seed,
                                       std::vector<Word>& labels) {
        std::vector<std::uint32_t> selected_count(graph.n, 0);
        std::vector<std::int8_t> state(graph.n, -1);
        std::vector<Vertex> order;
        order.reserve(graph.n);

        // At local bit b, selected_count is in [0,b], so b+1 buckets suffice.
        std::vector<std::vector<Vertex>> buckets(static_cast<std::size_t>(bit_count) + 1U);
        SplitMix64 rng(seed);

        for (std::uint32_t local_bit = 0; local_bit < bit_count; ++local_bit) {
            for (auto& bucket : buckets) bucket.clear();

            // Primary priority: vertices selected fewer times are processed first.
            for (Vertex v = 0; v < graph.n; ++v) {
                buckets[selected_count[v]].push_back(v);
            }

            // Secondary priority: reproducible random order inside equal-count buckets.
            order.clear();
            for (std::uint32_t count = 0; count <= local_bit; ++count) {
                auto& bucket = buckets[count];
                if (bucket.size() > 1U) {
                    std::shuffle(bucket.begin(), bucket.end(), rng);
                }
                order.insert(order.end(), bucket.begin(), bucket.end());
            }

            greedy_mis(graph, order, state);

            const Word bit = Word{1} << (bit_offset + local_bit);
            for (Vertex v = 0; v < graph.n; ++v) {
                if (state[v] == 1) {
                    labels[v] |= bit;
                    ++selected_count[v];
                }
            }
        }
    }
};

} // namespace lanegt
