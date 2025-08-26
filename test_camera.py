import pygame
import unittest
from Camera import Camera

class TestCamera(unittest.TestCase):
    def setUp(self):
        pygame.init()
        self.screen_width = 800
        self.screen_height = 600
        self.camera = Camera(self.screen_width, self.screen_height)
        self.camera.position = pygame.math.Vector2(0, 0) # Reset camera position for consistent tests
        self.camera.zoom_level = 1.0 # Reset zoom level

    def tearDown(self):
        pygame.quit()

    def test_point_on_screen_center(self):
        # A point at the center of the world should be at the center of the screen
        world_point = pygame.math.Vector2(0, 0)
        self.assertTrue(self.camera.is_point_on_screen(world_point))

    def test_point_on_screen_edge(self):
        # A point at the top-left corner of the screen
        world_point_top_left = self.camera.screen_to_world(pygame.math.Vector2(0, 0))
        self.assertTrue(self.camera.is_point_on_screen(world_point_top_left))

        # A point at the bottom-right corner of the screen
        world_point_bottom_right = self.camera.screen_to_world(pygame.math.Vector2(self.screen_width - 1, self.screen_height - 1))
        self.assertTrue(self.camera.is_point_on_screen(world_point_bottom_right))

    def test_point_off_screen(self):
        # A point far to the left
        world_point_left = self.camera.screen_to_world(pygame.math.Vector2(-100, self.screen_height / 2))
        self.assertFalse(self.camera.is_point_on_screen(world_point_left))

        # A point far to the right
        world_point_right = self.camera.screen_to_world(pygame.math.Vector2(self.screen_width + 100, self.screen_height / 2))
        self.assertFalse(self.camera.is_point_on_screen(world_point_right))

        # A point far above
        world_point_top = self.camera.screen_to_world(pygame.math.Vector2(self.screen_width / 2, -100))
        self.assertFalse(self.camera.is_point_on_screen(world_point_top))

        # A point far below
        world_point_bottom = self.camera.screen_to_world(pygame.math.Vector2(self.screen_width / 2, self.screen_height + 100))
        self.assertFalse(self.camera.is_point_on_screen(world_point_bottom))

    def test_point_with_margin(self):
        margin = 50
        # A point just outside the screen, but within the margin
        world_point_left_margin = self.camera.screen_to_world(pygame.math.Vector2(-margin + 1, self.screen_height / 2))
        self.assertTrue(self.camera.is_point_on_screen(world_point_left_margin, margin=margin))

        # A point just outside the margin
        world_point_left_outside_margin = self.camera.screen_to_world(pygame.math.Vector2(-margin - 1, self.screen_height / 2))
        self.assertFalse(self.camera.is_point_on_screen(world_point_left_outside_margin, margin=margin))

    def test_with_zoom(self):
        self.camera.zoom_level = 2.0
        # A point that would be on screen at zoom 1.0, but off screen at zoom 2.0 if camera position isn't adjusted
        # Let's test a point that should still be on screen
        world_point = pygame.math.Vector2(100, 100)
        self.assertTrue(self.camera.is_point_on_screen(world_point))

        # A point that should be off screen with zoom
        world_point_far = pygame.math.Vector2(self.screen_width, self.screen_height)
        self.assertFalse(self.camera.is_point_on_screen(world_point_far))

    def test_with_camera_position(self):
        self.camera.position = pygame.math.Vector2(100, 100)
        # A point that was at (0,0) world, now needs to be checked relative to new camera position
        world_point = pygame.math.Vector2(100, 100) # This point is now at the center of the screen
        self.assertTrue(self.camera.is_point_on_screen(world_point))

        world_point_off = pygame.math.Vector2(self.screen_width + 100, self.screen_height + 100)
        self.assertFalse(self.camera.is_point_on_screen(world_point_off))


if __name__ == '__main__':
    unittest.main()
