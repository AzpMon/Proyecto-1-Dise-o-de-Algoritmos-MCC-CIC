class Vertex:
    """
    Class representing a vertex in a graph.
    """
    def __init__(self, id):
        """
        Constructor for the Vertex class.
        Args:
            id (str): Unique identifier for the vertex.
        """
        self.id = id
        self.neighbors = []  # List of vertex neighbors
        self.grade = len(self.neighbors)


    def add_neighbor(self, neighbor):
        """
        Adds a neighbor to the vertex's neighbor list.
        
        Args:
            neighbor (Vertex): Neiprint(graph)ghbor vertex object.
        """
        self.neighbors.append(neighbor)

    def __str__(self):
        neighbors = ", ".join([n.id for n in self.neighbors])
        return f'Vertex({self.id}) -> [{neighbors}]'