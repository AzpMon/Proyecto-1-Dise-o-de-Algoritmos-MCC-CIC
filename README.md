# Graph Generator Toolkit

**Author:** Alejandro Monroy Azpeitia  
**Institution:** Centro de Investigación en Computación (CIC), Instituto Politécnico Nacional  
**Program:** Master's in Computer Science  
**Course:** Algorithm Design  

## Overview

This project implements a Python-based toolkit for creating, analyzing, and exporting various types of graphs. It includes custom object-oriented implementations for `Vertex`, `Edge`, and `Graph` classes. The graphs can be generated using classical random models and exported in DOT format for visualization (e.g., using Graphviz or Gephi).

The toolkit also includes graph traversal algorithms and a custom implementation of **Dijkstra's algorithm**, which builds a shortest-path tree and annotates each vertex with its distance to the source.

## Features

### ✔ Graph Representation

- Supports both **directed** and **undirected** graphs
- Custom `Vertex` and `Edge` classes using adjacency lists
- Vertex degrees and neighborhood tracking

### ✔ Graph Visualization

- Generates `.dot` files for visualization using tools such as **Graphviz** or **Gephi**
- Visual output with distance annotations when using Dijkstra

### ✔ Graph Algorithms

- **Breadth-First Search (BFS)** — returns a spanning tree
- **Depth-First Search (DFS)** — both recursive and iterative implementations
- **Dijkstra’s Algorithm**
  - Computes the shortest path tree from a source vertex
  - The resulting tree contains renamed vertices:  
    e.g., `node_2 (3.00)` indicates distance 3.0 from the source
  - All edge weights are assumed to be `1`

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
