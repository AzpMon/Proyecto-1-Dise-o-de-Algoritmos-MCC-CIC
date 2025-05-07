# Graph Generator Toolkit (Proyecto2)

**Author:** Alejandro Monroy Azpeitia  
**Institution:** Centro de Investigación en Computación (CIC), Instituto Politécnico Nacional  
**Program:** Master's in Computer Science  
**Course:** Algorithm Design  

## Overview

This project implements a Python-based toolkit for creating and exporting various types of undirected graphs, designed for experimentation and algorithmic analysis in graph theory. The implementation includes custom data structures for `Vertex`, `Edge`, and `Graph`, and supports the generation of several well-known random graph models. The graphs are exported in DOT format for visualization with tools such as Graphviz.

Additionally, implementations of **graph traversal algorithms**—Breadth-First Search (BFS), Depth-First Search (DFS) (both iterative and recursive variants)—have been added. These algorithms generate spanning trees from a selected initial node.

## Features

- **Graph Representation**
  - Undirected or directed graphs
  - Vertex and Edge classes with adjacency lists

- **DOT Export**
  - Generation of `.dot` files for external visualization

- **Graph Traversal Algorithms**
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS), both iterative and recursive implementations

- **Random Graph Generators**
  - Mesh grid graphs
  - Erdős–Rényi \( G(n, m) \)
  - Gilbert \( G(n, p) \)
  - Geographic random graphs \( G(n, r) \)
  - Barabási–Albert preferential attachment model
  - Dorogovtsev–Mendes growth model

- **Exported Outputs**
  - For the following graph types:
    - Mesh
    - Barabási–Albert
    - Dorogovtsev–Mendes
  - And for the following node sizes:
    - 30 nodes
    - 100 nodes
    - 500 nodes
  - The following outputs are generated:
    - Original graph images and `.dot` files
    - BFS tree images and `.dot` files
    - DFS tree (iterative) images and `.dot` files
    - DFS tree (recursive) images and `.dot` files

