# Graph Models in Python

This project implements several classical graph models using Python. The `Graph` class supports undirected and directed graphs, and includes methods for generating graphs using the following models:

- **Mesh (Grid) Graph**
- **Erdős–Rényi Graph (G(n, m))**
- **Gilbert Graph (G(n, p))**
- **Geographic Graph (G(n, r))**
- **Barabási–Albert Graph (Preferential Attachment)**
- **Dorogovtsev–Mendes Graph**

Each graph can be exported in DOT format for visualization using tools like Graphviz.

---

## Requirements

- Python 3.x
- [Graphviz](https://graphviz.org/) (for rendering `.dot` files)

No external Python libraries are required; the implementation uses only the standard library.

---

## File Structure

- `Vertex`: Class for vertices (nodes) in the graph.
- `Edge`: Class for edges between vertices.
- `Graph`: Main class that manages vertices, edges, and graph generation.

---

## Usage

You can generate and export different types of graphs by creating instances of the `Graph` class and calling the corresponding methods:

```
