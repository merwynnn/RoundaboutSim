import unittest
import pygame # Required for Vector2 and Rect
from Camera import Camera # Assuming Camera.py is in the same directory or accessible
from Constants import WIDTH, HEIGHT # Assuming these are available for camera initialization

class TestCamera(unittest.TestCase):

    def setUp(self):
        # Initialize Pygame's font module if any text rendering is involved in tested components,
        # though not directly for Camera it's good practice if other parts might use it.
        pygame.init()
        self.camera = Camera(WIDTH, HEIGHT)

    def test_camera_initialization(self):
        self.assertEqual(self.camera.position, pygame.math.Vector2(0, 0))
        self.assertEqual(self.camera.zoom_level, 1.0)
        self.assertEqual(self.camera.width, WIDTH)
        self.assertEqual(self.camera.height, HEIGHT)

    def test_apply_and_screen_to_world_no_pan_no_zoom(self):
        world_pos = pygame.math.Vector2(100, 150)
        screen_center = pygame.math.Vector2(WIDTH / 2, HEIGHT / 2)

        # When no pan and no zoom, apply should just offset by screen center
        # Corrected expectation for apply(): (pos - camera.pos) * zoom + screen_center
        self.assertEqual(self.camera.apply(world_pos), (world_pos - pygame.math.Vector2(0,0)) * 1.0 + screen_center)


        # Test screen_to_world
        # (screen_pos - screen_center) / zoom + camera_pos
        test_screen_pos = pygame.math.Vector2(WIDTH / 2 + 50, HEIGHT / 2 + 50) # 50,50 in world if no pan/zoom
        expected_world_pos = pygame.math.Vector2(50,50)
        # Use a tolerance for floating point comparisons for screen_to_world
        self.assertTrue(self.camera.screen_to_world(test_screen_pos).distance_to(expected_world_pos) < 1e-9)


    def test_camera_pan(self):
        self.camera.position = pygame.math.Vector2(50, -30)
        world_pos = pygame.math.Vector2(100, 100)
        screen_center = pygame.math.Vector2(WIDTH / 2, HEIGHT / 2)

        # Expected: (world_pos - camera.pos) * zoom + screen_center
        expected_screen_pos = (world_pos - self.camera.position) * self.camera.zoom_level + screen_center
        self.assertEqual(self.camera.apply(world_pos), expected_screen_pos)

        # Test screen_to_world with pan
        screen_pos_for_s2w = self.camera.apply(world_pos)
        # Use a tolerance for floating point comparisons
        self.assertTrue(self.camera.screen_to_world(screen_pos_for_s2w).distance_to(world_pos) < 1e-9)


    def test_camera_zoom(self):
        self.camera.zoom_level = 2.0
        world_pos = pygame.math.Vector2(100, 100)
        screen_center = pygame.math.Vector2(WIDTH / 2, HEIGHT / 2)

        # Expected: (world_pos - camera.pos) * zoom + screen_center
        expected_screen_pos = (world_pos - self.camera.position) * self.camera.zoom_level + screen_center
        self.assertEqual(self.camera.apply(world_pos), expected_screen_pos)

        # Test screen_to_world with zoom
        screen_pos_for_s2w = self.camera.apply(world_pos)
        self.assertTrue(self.camera.screen_to_world(screen_pos_for_s2w).distance_to(world_pos) < 1e-9)

    def test_camera_pan_and_zoom(self):
        self.camera.position = pygame.math.Vector2(-20, 30)
        self.camera.zoom_level = 0.5
        world_pos = pygame.math.Vector2(50, 50)
        screen_center = pygame.math.Vector2(WIDTH / 2, HEIGHT / 2)

        expected_screen_pos = (world_pos - self.camera.position) * self.camera.zoom_level + screen_center
        self.assertEqual(self.camera.apply(world_pos), expected_screen_pos)

        screen_pos_for_s2w = self.camera.apply(world_pos)
        self.assertTrue(self.camera.screen_to_world(screen_pos_for_s2w).distance_to(world_pos) < 1e-9)

    def test_get_scaled_value(self):
        self.camera.zoom_level = 1.5
        original_value = 100
        expected_scaled_value = original_value * 1.5
        self.assertEqual(self.camera.get_scaled_value(original_value), expected_scaled_value)

        self.camera.zoom_level = 0.5
        expected_scaled_value = original_value * 0.5
        self.assertEqual(self.camera.get_scaled_value(original_value), expected_scaled_value)

    def test_handle_event_zoom(self):
        initial_zoom = self.camera.zoom_level

        # Zoom in
        zoom_in_event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=4)
        self.camera.handle_event(zoom_in_event)
        self.assertAlmostEqual(self.camera.zoom_level, initial_zoom * 1.1) # Use assertAlmostEqual for float

        current_zoom = self.camera.zoom_level
        # Zoom out
        zoom_out_event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=5)
        self.camera.handle_event(zoom_out_event)
        self.assertAlmostEqual(self.camera.zoom_level, current_zoom / 1.1) # Use assertAlmostEqual

        # Test zoom out limit
        self.camera.zoom_level = 0.1
        self.camera.handle_event(zoom_out_event) # Try to zoom out further
        self.assertAlmostEqual(self.camera.zoom_level, 0.1) # Should stay at min limit

    def test_handle_event_panning_drag(self):
        self.camera.position = pygame.math.Vector2(0,0)
        self.camera.zoom_level = 1.0 # Ensure zoom is 1 for predictable pan
        start_mouse_pos = (WIDTH // 2, HEIGHT // 2)
        drag_event_start = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=start_mouse_pos)
        self.camera.handle_event(drag_event_start)
        self.assertTrue(self.camera.dragging)
        self.assertEqual(self.camera.drag_start_pos, pygame.math.Vector2(start_mouse_pos))

        moved_mouse_pos = (start_mouse_pos[0] + 50, start_mouse_pos[1] + 20)
        # pygame.MOUSEMOTION event requires 'buttons' (tuple of button states) and 'rel' (relative motion)
        drag_event_move = pygame.event.Event(pygame.MOUSEMOTION, buttons=(1,0,0), pos=moved_mouse_pos, rel=(50,20))
        self.camera.handle_event(drag_event_move)

        expected_camera_pos = pygame.math.Vector2(0,0) - (pygame.math.Vector2(moved_mouse_pos) - pygame.math.Vector2(start_mouse_pos)) / self.camera.zoom_level
        self.assertTrue(self.camera.position.distance_to(expected_camera_pos) < 1e-9)
        self.assertEqual(self.camera.drag_start_pos, pygame.math.Vector2(moved_mouse_pos))

        drag_event_end = pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=moved_mouse_pos)
        self.camera.handle_event(drag_event_end)
        self.assertFalse(self.camera.dragging)

    def tearDown(self):
        pygame.quit()

if __name__ == '__main__':
    unittest.main()
