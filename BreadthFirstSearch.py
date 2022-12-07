from queue import PriorityQueue
from math import inf
import networkx as nx
from scipy.spatial import distance

def breadthFirstSearchWithExplored(graph, start, end, pos):
    def findPath(prev, start, end):
        node = end
        path = []
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(node)
        path.reverse()
        return path

    frontier = [] 
    visitedNodes = []
    prev = {}  # The list of node parents
    visitedGraph = nx.Graph()
    graphList = list(nx.nodes(graph))
    frontier.append(start)
    visitedNodes.append(start)
    while(len(frontier) > 0):
        curNode = frontier.pop(0) 
        visitedGraph.add_node(graphList[curNode])

        if curNode in prev.keys():
            visitedGraph.add_edge(graphList[curNode], graphList[prev[curNode]])

        if curNode == end: # Break when we find the end node.
            break

        for neighbour in dict(graph.adjacency()).get(curNode):   # For all neighbours of the current Node.
            if neighbour not in visitedNodes:
                frontier.append(neighbour)
                visitedNodes.append(neighbour)
                prev[neighbour] = curNode




    # Path is a list of node indecies.
    # Visited Graph is a constructed NetworkX graph.
    return findPath(prev, start, end), visitedGraph