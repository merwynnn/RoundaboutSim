import pygame

class Camera:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.position = pygame.math.Vector2(0, 0)
        self.zoom_level = 1.0
        self.dragging = False
        self.drag_start_pos = pygame.math.Vector2(0, 0)

    def apply(self, pos):
        # Apply zoom first, then translation
        return (pos - self.position) * self.zoom_level + pygame.math.Vector2(self.width / 2, self.height / 2)

    def screen_to_world(self, screen_pos):
        # Reverse of apply: subtract screen center, divide by zoom, then add camera position
        return (screen_pos - pygame.math.Vector2(self.width / 2, self.height / 2)) / self.zoom_level + self.position

    def get_scaled_value(self, value):
       return value * self.zoom_level

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button for panning
                self.dragging = True
                self.drag_start_pos = pygame.math.Vector2(event.pos)
            elif event.button == 4:  # Mouse wheel up for zooming in
                self.zoom_level *= 1.1
            elif event.button == 5:  # Mouse wheel down for zooming out
                self.zoom_level /= 1.1
                if self.zoom_level < 0.1: # Prevent zooming out too much
                    self.zoom_level = 0.1
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                drag_current_pos = pygame.math.Vector2(event.pos)
                delta = drag_current_pos - self.drag_start_pos
                # Panning should be inverse to mouse movement and scaled by zoom
                self.position -= delta / self.zoom_level
                self.drag_start_pos = drag_current_pos
