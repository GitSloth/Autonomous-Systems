import tkinter as tk
from queue import PriorityQueue

class Game:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=600, height=600)
        self.canvas.pack()
        self.grid_size = 9
        self.cell_size = 60
        self.start = None
        self.end = None
        self.obstacles = []
        self.graph = self.build_graph()
        self.canvas.bind("<Button-1>", self.on_click)
        
        self.start_button = tk.Button(master, text="Start", command=self.run_game)
        self.start_button.pack(pady=10)
    
    def build_graph(self):
        graph = {}
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                vertex = (i, j)
                if vertex in self.obstacles:
                    continue
                neighbors = []
                if i > 0 and (i - 1, j) not in self.obstacles:
                    neighbors.append((i - 1, j))
                if i < self.grid_size - 1 and (i + 1, j) not in self.obstacles:
                    neighbors.append((i + 1, j))
                if j > 0 and (i, j - 1) not in self.obstacles:
                    neighbors.append((i, j - 1))
                if j < self.grid_size - 1 and (i, j + 1) not in self.obstacles:
                    neighbors.append((i, j + 1))
                graph[vertex] = neighbors
        return graph
    
    def draw_grid(self, path=None):
        self.canvas.delete("all")
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                color = "white"
                if (i, j) == self.start:
                    color = "blue"
                elif (i, j) == self.end:
                    color = "red"
                elif (i, j) in self.obstacles:
                    color = "black"
                elif path and (i, j) in path:
                    color = "green"
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color)
    
    def on_click(self, event):
        x = event.x // self.cell_size
        y = event.y // self.cell_size
        if not self.start:
            self.start = (y, x)
        elif not self.end:
            self.end = (y, x)
        else:
            self.obstacles.append((y, x))
        self.draw_grid()
    
    def calculate_shortest_path(self):
        distances = {vertex: float("inf") for vertex in self.graph}
        distances[self.start] = 0
        previous = {vertex: None for vertex in self.graph}
        queue = PriorityQueue()
        queue.put((0, self.start))
        
        while not queue.empty():
            current_distance, current_vertex = queue.get()
            if current_vertex == self.end:
                break
            if current_distance > distances[current_vertex]:
                continue
            for neighbor in self.graph[current_vertex]:
                distance = current_distance + 1  # All edges have weight 1
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_vertex
                    queue.put((distance, neighbor))
        
        path = []
        current = self.end
        while current != self.start:
            path.append(current)
            current = previous[current]
        path.append(self.start)
        path.reverse()
        return path
    
    def run_game(self):
        if not self.start or not self.end:
            return
        
        path = self.calculate_shortest_path()
        self.draw_grid(path)
        self.start_button.config(state=tk.DISABLED)

root = tk.Tk()
game = Game(root)
root.mainloop()
