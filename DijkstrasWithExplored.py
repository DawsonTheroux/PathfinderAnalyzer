from queue import PriorityQueue
from math import inf
import networkx as nx
from scipy.spatial import distance


def dijkstra_explored(graph, start, end, pos):
    def findPath(prev, start, end):
        node = end
        path = []
        while node != start:
            #print(node)
            path.append(node)
            node = prev[node]
        path.append(node) 
        path.reverse()
        return path
        
    def cost(u, v):
        return distance.euclidean(pos[u], pos[v])
        
    prev = {} 
    dist = {v: inf for v in list(nx.nodes(graph))} 
    visited = set() 
    pq = PriorityQueue()  
    visitedGraph = nx.Graph()
    graphList = list(nx.nodes(graph))
    dist[start] = 0  # dist from start -> start is zero
    pq.put((dist[start], start))
    
    while pq.qsize() != 0:
        curr_cost, curr = pq.get()
        visited.add(curr)
        visitedGraph.add_node(graphList[curr])
        if curr in prev.keys():
            visitedGraph.add_edge(graphList[curr], graphList[prev[curr]])

        if curr == end:
            return findPath(prev, start, end), visitedGraph

        for neighbor in dict(graph.adjacency()).get(curr):
            # The path cost to the current node.
            path = dist[curr] + cost(curr, neighbor)

            if path < dist[neighbor]:
                dist[neighbor] = path
                prev[neighbor] = curr
                if neighbor not in visited:
                    visited.add(neighbor)
                    pq.put((dist[neighbor],neighbor))
                else:
                    oldVersionOfNeighbout = pq.get((dist[neighbor],neighbor))
                    pq.put((dist[neighbor],neighbor))

    return None # Did not find the end node.