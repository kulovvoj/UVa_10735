# Turn the mixed graph into a directed graph by finding a graph, where each vertex has the same number of incoming and outgoing edges.
# Then apply normal euler path algorithm.
def make_graph(graph, V, E):
    cost_sum = [0]
    cost = [0 for _ in range(V)]
    dir_inc = [0 for _ in range(V)]
    dir_out = [0 for _ in range(V)]
    edges_undir = []

    def count_cost():
        for i in range(V):
            if (cost[i] + dir_out[i] - dir_inc[i]) % 2 == 1:
                return False
            cost[i] = (cost[i] + dir_out[i] - dir_inc[i]) // 2
            cost_sum[0] += cost[i]
        return True

    def make_residual(residual):
        vertices = 2 + len(edges_undir) + V
        for i in range(len(edges_undir)):
            residual[0][i + 1] = 1
            residual[i + 1][edges_undir[i][0] + 1 + len(edges_undir)] = 1
            residual[i + 1][edges_undir[i][1] + 1 + len(edges_undir)] = 1
        for i in range(V):
            if (cost[i] >= 0):
                residual[i + len(edges_undir) + 1][vertices - 1] = cost[i]
            else: 
                return False
        return True

    def BFS(residual, parent):
        vertices = 2 + len(edges_undir) + V
        visited = [False] * vertices
        queue = []

        # We add the outgoing starting node to the queue
        queue.append(0)
        visited[0] = True
        while(queue):
            current = queue.pop(0)

            for vertex, cost in enumerate(residual[current]):
                if (visited[vertex] == False and cost > 0):
                    if (vertex == vertices - 1):
                        parent[vertex] = current
                        return True
                    queue.append(vertex) 
                    visited[vertex] = True
                    parent[vertex] = current
        return False

    def ford_fulkerson(residual):
        vertices = 2 + len(edges_undir) + V
        parent = [-1] * (vertices)
        max_flow = 0
        while (BFS(residual, parent)):
            path_flow = 10000000000
            vertex = vertices - 1
            while (vertex != 0):
                path_flow = min(path_flow, residual[parent[vertex]][vertex])
                vertex = parent[vertex]
            max_flow += path_flow

            vertex = vertices - 1
            while (vertex != 0):
                residual[parent[vertex]][vertex] -= path_flow
                residual[vertex][parent[vertex]] += path_flow
                vertex = parent[vertex]
        return max_flow == cost_sum[0]

    for j in range(E):
        edge_from, edge_to, directionality = input().split()
        edge_from = int(edge_from)
        edge_to = int(edge_to)
        if directionality == "U":
            edge = [edge_from - 1, edge_to - 1]
            edges_undir.append(edge)
            cost[edge_to - 1] += 1
            cost[edge_from - 1] += 1
        else:
            #cost[edge_to - 1] += 1
            #cost[edge_from - 1] += 1
            dir_inc[edge_to - 1] += 1 
            dir_out[edge_from - 1] += 1
            graph[edge_from - 1].append(edge_to - 1)

    residual = [[0 for _ in range(2 + len(edges_undir) + V)] for _ in range(2 + len(edges_undir) + V)]

    if not count_cost(): 
        return False
    if not make_residual(residual):
        return False
    if cost_sum[0] != 0 and not ford_fulkerson(residual):
        return False

    for i in range(V):
        for j, edge in enumerate(residual[len(edges_undir) + i + 1]):
            if (edge == 1 and j > 0 and j < len(edges_undir) + 1):
                if (edges_undir[j - 1][0] != i):
                    graph[edges_undir[j - 1][0]].append(i)
                else:
                    graph[edges_undir[j - 1][1]].append(i)
    return True

def get_inout(graph, V, inout):
    for i in range(V):
        for j in graph[i]:
            inout[i][0] += 1
            inout[j][1] += 1

def find_start(inout):
    for i in range(len(inout)):
        if inout[i][0] > 0:
            return i

def get_euler_cycle(graph, inout, at, E, cycle):
    while inout[at][0] != 0:
        inout[at][0] -= 1
        next_vertex = graph[at][inout[at][0]]
        get_euler_cycle(graph, inout, next_vertex, E, cycle)
    
    cycle.insert(0, at)
    return len(cycle) == E + 1

def print_cycle(graph, V, E):
    cycle = []
    
    inout = [[0 for i in range(2)] for j in range(V)]
    get_inout(graph, V, inout)

    if get_euler_cycle(graph, inout, find_start(inout), E, cycle):
        for i in range(len(cycle)):
            cycle[i] += 1
        print(*cycle, sep = ' ')
        return

def main():
    iterations = int(input())
    for i in range(iterations):
        V, E = map(int, input().split())
        graph = [[] for _ in range(V)]
        if not make_graph(graph, V, E):
            print("No euler circuit exist")
            continue
        print_cycle(graph, V, E)
main()

