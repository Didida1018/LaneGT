# LaneGT

LaneGT is a compact, exact graph-query index based on **query–constraint co-routing**. Each unordered vertex pair is assigned a logical lane by vertex-ID XOR. Real edges constrain the same lane that a query would inspect; each lane independently builds a set of non-edge certificates. The packed certificate label uses **64 bits (8 bytes) per vertex**.

A certificate hit proves a pair is a **non-edge**. An inconclusive lookup falls back to the graph's exact adjacency lists, preserving correct answers.

## Features

- Construct lane-local independent-set certificates with a shared 64-bit label budget.
- Query exact edge existence using a lane certificate and CSR fallback.
- Count undirected triangles using degree-oriented forward adjacency and lane certificate pruning.

The command-line program supports 2, 4 or 8 lanes for both point queries and Triangle counting (4 by default). The core index also supports power-of-two lane counts dividing 64.

## Requirements

- C++17 compiler (GCC or Clang)
- Make
- Linux or macOS (or a C++17 build environment on Windows)

No third-party libraries are required.

## Build

```bash
make
make test
```

## Graph format

Input is a plain-text **undirected, zero-based** edge list. The first line contains the number of vertices and input edges, `n m`, followed by exactly `m` lines of `u v`:

```text
6 7
0 1
1 2
0 2
2 3
3 4
2 4
4 5
```

Vertex IDs must be in `[0, n)`. Self-loops and duplicate undirected edges are discarded. All other edges are treated as undirected. If the input uses one-based IDs, convert it to zero-based IDs before loading.

## Run

Build the index and answer an exact edge query:

```bash
./lanegt --graph examples/tiny_graph.txt --lanes 4 --query 0 2
./lanegt --graph examples/tiny_graph.txt --lanes 4 --query 0 5
```

A result is reported as `EDGE` or `NONEDGE`. The accompanying annotation identifies whether the certificate resolved the pair or exact CSR fallback was required.

Triangle counting:

```bash
./lanegt --graph examples/tiny_graph.txt --lanes 4 --triangles
```

The example has **2 triangles**. Query and triangle operations can be combined. Use `--audit` to check that no true edge is incorrectly certified as a non-edge:

```bash
./lanegt --graph examples/tiny_graph.txt --lanes 4 --query 0 5 --triangles --audit
```

## API

The header-only C++ components are available in `include/lanegt/`:

- `LaneGTCore.hpp`: index construction, lane routing, and non-edge certificate lookups.
- `Graph.hpp`: simple graph loader, sorted CSR adjacency, and exact edge queries.
- `Triangle.hpp`: exact triangle counting with lane-local class-pair pruning.

The index builder builds its own lane-local CSR structures; the original graph CSR remains available for fallback queries. The triangle implementation uses the same LaneGT certificates and returns exact counts.
