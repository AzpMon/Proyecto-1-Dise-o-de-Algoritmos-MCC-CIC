# Graph Generator Toolkit

**Author:** Alejandro Monroy Azpeitia  
**Institution:** Centro de Investigación en Computación (CIC), Instituto Politécnico Nacional  
**Program:** Master's in Computer Science  
**Course:** Algorithm Design  

## Overview

This project implements a Python-based toolkit for creating, analyzing, and exporting various types of graphs. It includes custom object-oriented implementations for `Vertex`, `Edge`, and `Graph` classes. Graphs can be generated using classical random models and exported in DOT format for visualization (e.g., using Graphviz or Gephi).

The toolkit also includes graph traversal algorithms and a custom implementation of **Dijkstra's algorithm**, as well as **Minimum Spanning Tree (MST)** algorithms such as **Kruskal**, **Reverse-Delete (Kruskal Inverse)**, and **Prim**.

A new script has been added to **automatically generate and analyze graphs** of different types and sizes. For each graph:
- Three MST algorithms are executed: Kruskal, Kruskal Inverse, and Prim.
- The DOT file of each result is saved.
- The name of the generator, algorithm used, and total MST weight are printed.

## Features

### ✔ Graph Representation

- Supports both **directed** and **undirected** graphs
- Custom `Vertex` and `Edge` classes using adjacency lists
- Vertex degree and neighborhood tracking

### ✔ Graph Visualization

- Generates `.dot` files for visualization using tools such as **Graphviz** or **Gephi**
- Visual output with distance annotations when using Dijkstra

### ✔ Graph Algorithms

- **Breadth-First Search (BFS)** — returns a spanning tree
- **Depth-First Search (DFS)** — both recursive and iterative implementations
- **Dijkstra’s Algorithm**
  - Computes the shortest-path tree from a source vertex
  - Vertices in the result include their distance: e.g., `node_2 (3.00)` means distance 3.0
  - Assumes unit weights on edges
- **Minimum Spanning Tree (MST) Algorithms**
  - **Kruskal's Algorithm**
  - **Reverse-Delete Algorithm** (Kruskal Inverse)
  - **Prim's Algorithm**
  - Automatically tested across all graph models and saved to DOT format

### ✔ Random Graph Generators

- `meshGraph(rows, cols)` — structured mesh grids
- `ErdosRenyiGraph(n, m)` — Erdős–Rényi model \( G(n, m) \)
- `GilbertGraph(n, p)` — Gilbert model \( G(n, p) \)
- `geographicGraph(n, r)` — Geographic proximity model \( G(n, r) \)
- `BarabasiAlbertGraph(n, d)` — Barabási–Albert preferential attachment
- `DorogovtsevMendesGraph(n)` — Triadic closure growth model

## Example: Running Dijkstra

```python
g = Graph()
g.meshGraph(3, 3)  # Generates a 3x3 grid
shortest_path_tree = g.Dijkstra("0_0")
shortest_path_tree.generate_dot("dijkstra_tree.dot")
