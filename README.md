# Graph Generator Toolkit

**Author:** Alejandro Monroy Azpeitia  
**Institution:** Centro de Investigación en Computación (CIC), Instituto Politécnico Nacional  
**Program:** Master's in Computer Science  
**Course:** Algorithm Design  

## Overview

This project implements a Python-based toolkit for creating and exporting various types of undirected graphs, designed for experimentation and algorithmic analysis in graph theory. The implementation includes custom data structures for `Vertex`, `Edge`, and `Graph`, and supports the generation of several well-known random graph models. The graphs are exported in DOT format for visualization with tools such as Graphviz.

## Features

- **Graph Representation**
  - Undirected or directed graphs
  - Vertex and Edge classes with adjacency lists
- **DOT Export**
  - Generation of `.dot` files for external visualization
- **Random Graph Generators**
  - Mesh grid graphs
  - Erdős–Rényi \( G(n, m) \)
  - Gilbert \( G(n, p) \)
  - Geographic random graphs \( G(n, r) \)
  - Barabási–Albert preferential attachment model
  - Dorogovtsev–Mendes growth model
