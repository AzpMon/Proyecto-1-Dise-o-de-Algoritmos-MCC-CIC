import heapq
from Vertex import Vertex
from Edge import Edge
import random
import math
import collections
import itertools

class Graph:
    """
    Class representing a graph with vertices and edges.
    """
    def __init__(self, directed=False):
        """
        Constructor for the Graph class.
        Args:
            directed (bool): Defines whether the graph is directed.
        """
        self.vertices = {}  # Dictionary of vertices
        self.edges = {}  # Dictionary of edges
        self.directed = directed

        
        self.typesOfGraphs = {'mesh':self.meshGraph, 'ErdosRenyi':self.ErdosRenyiGraph, 
                              'Gilbert':self.GilbertGraph, 'geographic':self.geographicGraph, 
                              'BarabasiAlbert':self.BarabasiAlbertGraph, 
                              'DorogovtsevMendes':self.DorogovtsevMendesGraph
                              }

    def add_vertex(self, id):
        """
        Adds a vertex to the graph.
        
        Args:
            id (str): Vertex identifier.
        """
        if id not in self.vertices:
            self.vertices[id] = Vertex(id)


    def total_weight(self):
        return sum(edge.weight for edge in self.edges.values())


    def add_edge(self, source_id, target_id, weight=1):
        if source_id not in self.vertices:
            self.add_vertex(source_id)
        if target_id not in self.vertices:
            self.add_vertex(target_id)

        source = self.vertices[source_id]
        target = self.vertices[target_id]

        edge_id = f'{source_id}->{target_id}' if self.directed else f'{source_id}--{target_id}'
        self.edges[edge_id] = Edge(source, target, edge_id, self.directed, weight)

        source.add_neighbor(target)
        if not self.directed:
            target.add_neighbor(source)

        
    def gradeVertex(self, id):
        vertex = self.vertices.get(id)

        if not vertex:
            return 0
    
        else:
            return len(vertex.neighbors)

    def __str__(self):
        graph_type = 'Directed' if self.directed else 'Undirected'
        vertices = ", ".join(self.vertices.keys())
        edges = ", ".join(str(edge) for edge in self.edges.values())
        return f'Graph ({graph_type}):\n  Vertices = [{vertices}]\n  Edges = [{edges}]'
    

    def generate_dot(self, filename='graph.dot'):
        """
        Generates a DOT format file for the graph.
        
        Args:
            filename (str): Output file name.
        """
        dot_graph = "digraph G {\n" if self.directed else "graph G {\n"
        
        for vertex in self.vertices.values():
            dot_graph += f'    "{vertex.id}";\n'

        for edge in self.edges.values():
            connector = " -> " if edge.directed else " -- "
            dot_graph += f'    "{edge.source.id}"{connector}"{edge.target.id}";\n'
        #
        dot_graph += "}"
        
        with open(filename, 'w') as f:  
            f.write(dot_graph)
        
        

    def meshGraph(self, numRows, numColumns):
            """
            Creates a mesh graph with specified rows and columns.
            
            Args:
                rows (int): Number of rows in the mesh.
                columns (int): Number of columns in the mesh.
            """
            for row in range(numRows):
                for column in range(numColumns - 1):
                    source = f'{row}_{column}'
                    target_horizontal = f'{row}_{column + 1}'
                    self.add_edge(source, target_horizontal)

            for column in range(numColumns):
                for row in range(numRows - 1):
                    source = f'{row}_{column}'
                    target_vertical = f'{row + 1}_{column}'
                    self.add_edge(source, target_vertical)



    def ErdosRenyiGraph(self, numVertices, numEdges):
        """
        Generates an Erdős–Rényi random graph G(n, m) as a method of the Graph class.

        Args:
            n (int): Number of vertices.
            m (int): Number of edges.
        """
        # Vertex creation        
        for vertice in range(numVertices):
            self.add_vertex(str(vertice))

        # 
        ids = list(self.vertices.keys())
        possibleEdges = list(itertools.combinations(ids, 2)) 

        selectedEdges = random.sample(possibleEdges, numEdges)


        for source_id, target_id in selectedEdges:
            self.add_edge(source_id, target_id)



    def GilbertGraph(self, numVertices, prob ):
        """_summary_

        Args:
            numVertices (int): Number of vertices
            prob (float): Probability associated to create an edge
        """

        # Vertex Creation
        for i in range(numVertices):
            self.add_vertex(str(i))
        ids = list(self.vertices.keys())

        # Create all the possible edge pairs
        posiblesEdges = itertools.combinations(ids, 2)

        # Para cada par, lanzar una moneda con probabilidad `p`
        for source_id, target_id in posiblesEdges:
            if random.random() <= prob:
                self.add_edge(source_id, target_id)



    def geographicGraph(self, n, r):
        """
        Generates a random graph using the simple geographic model G(n, r).
        
        Args:
            n (int): Number of nodes (> 0).
            r (float): Maximum distance to create an edge (0 < r <= 1).
        """



        # Assign random positions (x, y) in [0,1] x [0,1]
        positions = {}
        for i in range(n):
            node_id = str(i)
            self.add_vertex(node_id)
            positions[node_id] = (random.random(), random.random())

        ids = list(positions.keys())

        # Check each pair and create an edge if distance ≤ r
        for i in range(n):
            for j in range(i + 1, n) if not self.directed else range(n):
                if not self.directed and i >= j:
                    continue
                source_id = ids[i]
                target_id = ids[j]
                x1, y1 = positions[source_id]
                x2, y2 = positions[target_id]
                distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

                if distance <= r:
                    self.add_edge(source_id, target_id)

    def BarabasiAlbertGraph(self, numVertices, maxGrade):
        # Iterate through the number of vertices to be added
        for vertex in range(numVertices):
            # Add the new vertex to the graph
            self.add_vertex(str(vertex))

            # Try to connect the new vertex to existing ones
            for existingVertexKey in self.vertices.keys():
                # Avoid self-loops and check max degree constraint
                if existingVertexKey != str(vertex) and self.gradeVertex(existingVertexKey) < maxGrade:
                    # Compute probability based on inverse of current degree
                    probsCreateGraph = 1 - (self.gradeVertex(existingVertexKey)) / maxGrade
                    randomProb = random.random()

                    # Add edge with the computed probability
                    if randomProb < probsCreateGraph:
                        self.add_edge(str(vertex), existingVertexKey)



    def DorogovtsevMendesGraph(self, numVertex):
        # Require at least 3 vertices to start the base triangle
        if numVertex < 3:
            raise ValueError("Number of vertices must be at least 3.")
        
        # Create the initial triangle
        self.add_vertex("0")    
        self.add_vertex("1")
        self.add_vertex("2")

        self.add_edge("0", "1")
        self.add_edge("1", "2")
        self.add_edge("2", "0")
        
        # Add new vertices one by one
        for i in range(3, numVertex):
            # Choose a random existing vertex
            randomVertex1Key = random.choice(list(self.vertices.keys()))
            randomVertex1 = self.vertices[randomVertex1Key]
            
            # Choose one of its neighbors
            randomVertex2 = random.choice(list(randomVertex1.neighbors))
            
            # Add the new vertex
            self.add_vertex(str(i))
            
            # Connect it to both endpoints of the chosen edge
            self.add_edge(str(i), randomVertex1.id)
            self.add_edge(str(i), randomVertex2.id)
            
            
 
    def BFS(self, initVertex):
        """Performs a Breadth-First Search (BFS) starting from a given vertex and constructs
        a BFS tree (i.e., a spanning tree without cycles that connects all reachable vertices).
        Args:
            initVertex (Vertex): The initial vertex where the algorithm will begins.

        Returns:
            Graph: A new Graph object representing the BFS tree rooted at initVertex.
            If initVertex does not exist in the graph, an empty list is returned
        """
        
        
        # Se crea el grafo BFS_tree
        BFS_tree = Graph()
        
        
        # Se verifica si el initVertex existe en los vertices
        initVertex = self.vertices.get(str(initVertex))   
        if not initVertex:
            return []   # Si el initV   ertex no existe
        
        
        # Se inicializa el algoritmo de BFS
        queueVertex = collections.deque()
        queueVertex.append(initVertex)
        
        visitedVertices = set()
        visitedVertices.add(initVertex)
        
        
        while queueVertex: # Mientras no sea vacia queueVertex
            
            actualVertex = queueVertex.popleft()
            for neighbor in actualVertex.neighbors:
                if neighbor not in visitedVertices:
                    queueVertex.append(neighbor)
                    visitedVertices.add(neighbor)
                    BFS_tree.add_edge(actualVertex, neighbor)

        return BFS_tree
    
    
    def dfsRecurisive(self, initVertex):
        initVertexObj =  self.vertices.get(str(initVertex))   
        if not initVertexObj:
            return Graph(directed=self.directed)  # Retorna un grafo vacío si el vértice no existe

        visitedVertices = set()
        dfsTree = Graph(directed=self.directed)
        dfsTree.add_vertex(initVertex)

        def dfs(vertex):
            visitedVertices.add(vertex)
            for neighbor in vertex.neighbors:
                if neighbor not in visitedVertices:
                    dfsTree.add_vertex(neighbor.id)
                    dfsTree.add_edge(vertex.id, neighbor.id)
                    dfs(neighbor)

        dfs(initVertexObj)
        return dfsTree
        
    def dfsIterative(self, initVertex):
        # Obtener el objeto del vértice inicial
        initVertexObj =  self.vertices.get(str(initVertex))   
        if not initVertexObj:
            return []  # Si el vértice inicial no existe, retorna lista vacía

        # Grafo que representará el árbol DFS
        dfs_tree = Graph()
        dfs_tree.add_vertex(initVertex)

        visited = set()
        stack = [initVertexObj]
        visited.add(initVertexObj)

        while stack:
            current = stack.pop()
            for neighbor in current.neighbors:
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)

                    # Añadir vértices y arista al árbol DFS
                    dfs_tree.add_vertex(neighbor.id)
                    dfs_tree.add_edge(current.id, neighbor.id)

        return dfs_tree


    def is_connected(self):
        if not self.vertices:
            return True
        start = next(iter(self.vertices))  # Tomar cualquier vértice para empezar
        visited = set()
        queue = [start]
        while queue:
            current = queue.pop(0)
            if current not in visited:
                visited.add(current)
                # Obtener vecinos del vértice current
                vecinos = []
                for edge in self.edges.values():
                    if edge.source == current:
                        vecinos.append(edge.target)
                    elif edge.target == current:
                        vecinos.append(edge.source)
                for vecino in vecinos:
                    if vecino not in visited:
                        queue.append(vecino)
        return len(visited) == len(self.vertices)


    def kruskal_mst(self):
        parent = {}
        rank = {}

        def find(v):
            if parent[v] != v:
                parent[v] = find(parent[v])
            return parent[v]

        def union(u, v):
            root_u = find(u)
            root_v = find(v)
            if root_u == root_v:
                return False
            if rank[root_u] > rank[root_v]:
                parent[root_v] = root_u
            else:
                parent[root_u] = root_v
                if rank[root_u] == rank[root_v]:
                    rank[root_v] += 1
            return True

        for vertex in self.vertices.values():
            parent[vertex.id] = vertex.id
            rank[vertex.id] = 0

        sorted_edges = sorted(self.edges.values(), key=lambda e: e.weight)

        mst = Graph(directed=self.directed)
        total_weight = 0
        for v_id in self.vertices:
            mst.add_vertex(v_id)

        for edge in sorted_edges:
            u = edge.source.id
            v = edge.target.id
            if union(u, v):
                mst.add_edge(u, v, edge.weight)
                total_weight += edge.weight

        print(f"Peso total del MST (Kruskal directo): {total_weight}")
        return mst



    def kruskal_inverse_mst(self):
        current_edges = list(self.edges.values())
        current_edges.sort(key=lambda e: e.weight, reverse=True)

        def is_connected(edge_list):
            test_graph = Graph(directed=self.directed)
            for v_id in self.vertices:
                test_graph.add_vertex(v_id)
            for e in edge_list:
                test_graph.add_edge(e.source.id, e.target.id, e.weight)
            start = next(iter(test_graph.vertices))
            bfs_tree = test_graph.BFS(start)
            return len(bfs_tree.vertices) == len(self.vertices)

        for edge in current_edges[:]:
            temp = [e for e in current_edges if e != edge]
            if is_connected(temp):
                current_edges = temp

        mst = Graph(directed=self.directed)
        total_weight = 0
        for v_id in self.vertices:
            mst.add_vertex(v_id)
        for edge in current_edges:
            mst.add_edge(edge.source.id, edge.target.id, edge.weight)
            total_weight += edge.weight

        print(f"Peso total del MST (Kruskal inverso): {total_weight}")
        return mst

    def prim_mst(self):
        """
        Algoritmo de Prim para construir el Árbol de Expansión Mínima (MST).
        Retorna un nuevo objeto Graph con las aristas del MST.
        """
        if not self.vertices:
            return Graph(directed=self.directed)

        mst = Graph(directed=self.directed)
        visited = set()

        start_vertex = next(iter(self.vertices.values()))
        visited.add(start_vertex.id)
        mst.add_vertex(start_vertex.id)

        # Cola de prioridad: (peso, id_origen, id_destino, edge)
        edges_heap = []

        for edge in self.edges:
            if edge.source.id == start_vertex.id or (not edge.directed and edge.target.id == start_vertex.id):
                neighbor = edge.target if edge.source.id == start_vertex.id else edge.source
                heapq.heappush(edges_heap, (edge.weight, start_vertex.id, neighbor.id, edge))

        while edges_heap and len(visited) < len(self.vertices):
            weight, u_id, v_id, edge = heapq.heappop(edges_heap)

            if v_id in visited and u_id in visited:
                continue

            new_vertex_id = v_id if v_id not in visited else u_id

            visited.add(new_vertex_id)
            mst.add_vertex(new_vertex_id)
            mst.add_edge(u_id, v_id, weight)

            # Agregar nuevas aristas desde el nuevo vértice a la cola
            for e in self.edges:
                if ((e.source.id == new_vertex_id and e.target.id not in visited) or
                    (not self.directed and e.target.id == new_vertex_id and e.source.id not in visited)):
                    neighbor = e.target if e.source.id == new_vertex_id else e.source
                    heapq.heappush(edges_heap, (e.weight, e.source.id, e.target.id, e))

        return mst
