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

    def add_edge(self, source_id, target_id):
        """
        Adds an edge between two vertices.
        
        Args:
            source_id (str): ID of the source vertex.
            target_id (str): ID of the target vertex.
        """
        if source_id not in self.vertices:
            self.add_vertex(source_id)
        if target_id not in self.vertices:
            self.add_vertex(target_id)

        source = self.vertices[source_id]
        target = self.vertices[target_id]

        edge_id = f'{source_id}<->{target_id}' if self.directed else f'{source_id}--{target_id}'
        self.edges[edge_id] = Edge(source, target, edge_id, self.directed)
        
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
        
        dot_graph += "}"
        
        with open(filename, 'w') as f:  # Corrección aquí
            f.write(dot_graph)
        
        print(f'DOT file saved as {filename}')
        

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