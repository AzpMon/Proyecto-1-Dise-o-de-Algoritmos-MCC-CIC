class Edge:
    """
    Class representing an edge in a graph.
    """
    def __init__(self, source, target, id, directed=False):
        """
        Constructor for the Edge class.
        Args:
            source (Vertex): Source vertex.
            target (Vertex): Target vertex.
            id (str): Unique identifier for the edge.
            directed (bool): Defines whether the edge is directed.
        """
        self.source = source
        self.target = target
        self.id = id
        self.directed = directed

    def __str__(self):
        direction = "<->" if self.directed else "--"
        return f'{self.source.id}{direction}{self.target.id}'