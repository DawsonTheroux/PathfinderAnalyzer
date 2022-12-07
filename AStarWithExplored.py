from queue import PriorityQueue
from math import inf
import networkx as nx
from scipy.spatial import distance


def aStarWithExplored(graph, start, end, heuristic, pos):
    def findPath(prev, start, end):
        node = end
        path = []
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(node)
        path.reverse()
        return path

    visited = set()
    prev = dict()
    dist = {v: inf for v in list(nx.nodes(graph))} 
    dist[start] = 0
    frontier = PriorityQueue()
    frontier.put((dist[start], start))
    graphList = list(nx.nodes(graph))

    visitedGraph = nx.Graph() # The graph containing all the edges and nodes visited
    while frontier.qsize() > 0:
        curr_cost, curnode = frontier.get()

        if curnode in visited:
            continue

        if curnode == end:
            return findPath(prev, start, end), visitedGraph

        visited.add(curnode)
        visitedGraph.add_node(graphList[curnode])

        if curnode in prev.keys():
            visitedGraph.add_edge(graphList[curnode], graphList[prev[curnode]])

        for neighbour in dict(graph.adjacency()).get(curnode):   # For all neighbours of the current Node.
            frontier.put((dist[curnode] + distance.euclidean(pos[curnode], pos[neighbour]) + heuristic(pos[neighbour], pos[end]), neighbour))

            pathcost = dist[curnode] + distance.euclidean(pos[curnode], pos[neighbour]) 
            if pathcost < dist[neighbour]:
                dist[neighbour] = pathcost
                prev[neighbour] = curnode

    return findPath(prev, start, end), visitedGraph