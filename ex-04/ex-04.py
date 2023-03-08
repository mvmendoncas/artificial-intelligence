from heapq import heappop, heappush # Biblioteca para implementação de filas de prioridade
from math import sqrt # Função para calcular raiz quadrada

class Graph:
    def __init__(self):
        self.nodes = set() # Conjunto de nós do grafo
        self.edges = {} # Dicionário que mapeia cada nó às suas arestas adjacentes
        self.cost = {} # Dicionário que mapeia cada aresta a seu custo
        
    def add_node(self, value):
        self.nodes.add(value) # Adiciona um nó ao conjunto de nós do grafo
        self.edges[value] = [] # Inicializa o dicionário de arestas vazio para esse nó
        
    def add_edge(self, from_node, to_node, cost):
        self.edges[from_node].append(to_node) # Adiciona um nó destino à lista de arestas do nó de origem
        self.cost[(from_node, to_node)] = cost # Mapeia a aresta ao seu custo
        
    def heuristic(self, n):
        x1, y1 = n
        x2, y2 = self.goal
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # Heurística: distância Euclidiana entre o nó atual e o nó objetivo
        
    def a_star_search(self, start, goal):
        self.start = start # Define o nó inicial como um atributo da classe
        self.goal = goal # Define o nó objetivo como um atributo da classe
        frontier = [] # Fila de prioridade de nós a serem explorados, ordenados pelo custo estimado
        heappush(frontier, (0, start)) # Adiciona o nó inicial à fila, com custo zero
        came_from = {} # Dicionário que mapeia cada nó visitado ao seu nó anterior no menor caminho encontrado
        cost_so_far = {} # Dicionário que mapeia cada nó visitado ao custo acumulado até aquele nó no menor caminho encontrado
        came_from[start] = None # O nó inicial não tem nó anterior
        cost_so_far[start] = 0 # O custo acumulado do nó inicial é zero
        
        while frontier:
            current = heappop(frontier)[1] # Seleciona o nó com menor custo estimado
            
            if current == goal: # Se o nó selecionado é o objetivo, termina a busca
                break
                
            for next_node in self.edges[current]: # Para cada nó adjacente ao nó selecionado
                new_cost = cost_so_far[current] + self.cost[(current, next_node)] # Calcula o custo acumulado até esse nó
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]: # Se o nó ainda não foi visitado ou o novo caminho é melhor
                    cost_so_far[next_node] = new_cost # Atualiza o custo acumulado do nó
                    priority = new_cost + self.heuristic(next_node) # Calcula a estimativa do custo total para chegar ao objetivo passando pelo nó
                    heappush(frontier, (priority, next_node)) # Adiciona o nó à fila de prioridade com sua nova estimativa de custo
                    came_from[next_node] = current  # Mapeia o nó visitado ao seu nó anterior no menor caminho encontrado
                    
        return came_from, cost_so_far


#Exemplo de uso 

graph = Graph()
graph.add_node((0, 0))
graph.add_node((1, 0))
graph.add_node((2, 0))
graph.add_node((2, 1))
graph.add_node((2, 2))
graph.add_node((1, 2))
graph.add_node((0, 2))
graph.add_edge((0, 0), (1, 0), 1)
graph.add_edge((1, 0), (2, 0), 1)
graph.add_edge((2, 0), (2, 1), 1)
graph.add_edge((2, 1), (2, 2), 1)
graph.add_edge((2, 2), (1, 2), 1)
graph.add_edge((1, 2), (0, 2), 1)
graph.add_edge((0, 2), (0, 0), 1)

start = (0, 0)
goal = (2, 2)


came_from, cost_so_far = graph.a_star_search(start, goal)
print("Caminho:", end=" ")
node = goal
while node != start:
    print(node, end=" ")
    node = came_from[node]
print(start)

print("Custo total:", cost_so_far[goal])

#Rsultado
#Caminho: (2, 2) (2, 1) (2, 0) (1, 0) (0, 0)
#Custo total: 4

"""
Isso significa que o menor caminho entre os pontos (0, 0) e (2, 2) 
no grafo fornecido tem custo total de 4, e é dado pelos pontos (0, 0), 
(0, 2), (1, 2), (2, 1) e (2, 2).
"""







