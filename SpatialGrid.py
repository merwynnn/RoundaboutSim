import math

class SpatialGrid:
    def __init__(self, width, height, cell_size):
        self.cell_size = cell_size
        self.grid_width = math.ceil(width / cell_size)
        self.grid_height = math.ceil(height / cell_size)
        self.grid = [[] for _ in range(self.grid_width * self.grid_height)]

    def _get_cell_index(self, pos):
        x = int(pos.x / self.cell_size)
        y = int(pos.y / self.cell_size)
        return x, y

    def add_car(self, car):
        x, y = self._get_cell_index(car.pos)
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            self.grid[y * self.grid_width + x].append(car)

    def get_cars_in_cell(self, x, y):
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            return self.grid[y * self.grid_width + x]
        return []

    def get_cars_in_neighborhood(self, car):
        cars_in_neighborhood = []
        car_x, car_y = self._get_cell_index(car.pos)

        for i in range(-1, 2):
            for j in range(-1, 2):
                cars_in_neighborhood.extend(self.get_cars_in_cell(car_x + i, car_y + j))
        
        return cars_in_neighborhood

    def clear(self):
        for cell in self.grid:
            cell.clear()

    def update(self, cars):
        self.clear()
        for car in cars:
            self.add_car(car)
