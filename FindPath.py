import heapq
import Tkinter
import random
import time
class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = set()
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def add2wall(self, id):
        self.walls.add(id)
        
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

    def cost(self, current, goal):
        return 1
    
    def clear_wall(self):
        self.walls.clear()
    
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
    def get_smallest(self):
        return self.elements[0][0]

def draw_grid (row,column,wall,path):

    C = Tkinter.Canvas(top, bg="white", height=column*10, width=row*10)

    for i in range(row):
        line = C.create_line(0,i*10,column*10,i*10,fill="green",width=1)

    for j in range(column):
        line = C.create_line(j*10,0,j*10,row*10,   fill="green",width=1)

    for(x,y) in wall:
        rect = C.create_rectangle(y*10,x*10,y*10+10,x*10+10,fill='red')

    for(x,y) in path:
        rect = C.create_rectangle(y*10,x*10,y*10+10,x*10+10,fill='yellow')
    C.pack()
    top.mainloop()


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def ComputePath():    
    
    cost_so_far[start] = 0
    #print frontier.get_smallest()
    while not frontier.empty() and cost_so_far[goal] > frontier.get_smallest():
    #while not frontier.empty():
        current = frontier.get()
        #print current
##        if current == goal:
##            print 'Im here'
##            break
        for state in graph.neighbors(current):
            if search[state] < counter:
                cost_so_far[state] = float('inf')
                search[state] = counter
            new_cost = cost_so_far[current] + graph.cost(current, state)
            if state not in cost_so_far or new_cost < cost_so_far[state]:
                cost_so_far[state] = new_cost
                priority = (new_cost + heuristic(state, goal)) * c - new_cost 
                frontier.put(state, priority)
                came_from[state] = current

graph = SquareGrid(100, 100)
unvisited = set()
visited = set()

#set all of the cells as unvisited and unblocked
for i in range(graph.width):
    for j in range(graph.height):
        unvisited.add((i, j))
while unvisited:
    ran_grid = unvisited.pop()
    stack = [ran_grid]
    while stack:
        grid = stack.pop()
        if grid not in visited:
            visited.add(grid)
            grid_neighbors = graph.neighbors(grid)
            for neighbor in grid_neighbors:
                if (neighbor not in visited):
                    if random.random() > 0.2:
                        stack.append(neighbor)
                    else:
                        graph.add2wall(neighbor)
    unvisited = unvisited - visited - set(graph.walls)
unvisited = visited - graph.walls

start = random.sample(unvisited, 1)[0]
goal = random.sample(unvisited, 1)[0]
barriers = graph.walls.copy()
graph.clear_wall()
print 'start and goal is ', start, goal
path = []
search = {}
flag = False
counter = 0
for i in range(graph.width):
    for j in range(graph.height):
        search[(i, j)] = 0
cost_so_far = {}
c = graph.width + graph.height
while start != goal:
    counter += 1
    cost_so_far[start] = 0
    search[start] = 0
    cost_so_far[goal] = float('inf')
    search[goal] = 0
    frontier = PriorityQueue()
    close = PriorityQueue()
    frontier.put(start, cost_so_far[start] + heuristic(start, goal))
    came_from = {}
    tree = []
    came_from[start] = None
    ComputePath()
    if frontier.empty():
        print "I cannot reach the target."
        flag = True 
        break
    tree.append(goal)
    back = goal
    while True:
        back = came_from[back]
        #print (back),
        tree.append(back)
        if back == start:
            break
    
    #rint tree
    for i in range(len(tree)-1, -1, -1):
        start = tree[i]
        if start not in path:
            path.append(start)
        for grid in graph.neighbors(start):
            if grid in barriers:
                graph.add2wall(grid)
        if tree[i-1] in barriers:
            break
#print graph.walls
if not flag:
    print "I reached"
    #print path
top = Tkinter.Tk()
draw_grid(graph.width,graph.height,barriers,path)


