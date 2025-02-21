from unittest import TestCase
from numpy import pi
from numpy import cos, sin

from src.arm.ArmController import ArmController


class TestArmController(TestCase):

    # expected robot pose:
    #     __________
    #     |
    #     |
    #     |
    #     |
    # ----------- y-->
    def test1(self):
        config = TestConfig(0, 10, 10, 5)

        controller = ArmController(config)

        angles = controller.run(0, 15, 10, 0)
        self.assertAlmostEqual(pi / 2, angles[0], places=2, msg='got a wrong value for angle 1')
        self.assertAlmostEqual(pi / 2, angles[1], places=2, msg='got a wrong value for angle 2')
        self.assertAlmostEqual(-pi / 2, angles[2], places=2, msg='got a wrong value for angle 3')
        self.assertAlmostEqual(0, angles[3], places=2, msg='got a wrong value for angle 4')

    # expected robot pose:
    #     _____
    #     |    \
    #     |     \
    #     |
    #     |
    # ----------- y-->
    def test2(self):

        a2 = 10
        d4 = 10
        d6 = 10
        config = TestConfig(0, a2, d4, d6)

        controller = ArmController(config)

        phi = -pi/4
        x = 0
        y = d4 + d6 * cos(phi)
        z = a2 + d6 * sin(phi)

        angles = controller.run(x, y, z, phi)
        self.assertAlmostEqual(pi / 2, angles[0], places=2, msg='got a wrong value for angle 1')
        self.assertAlmostEqual(pi / 2, angles[1], places=2, msg='got a wrong value for angle 2')
        self.assertAlmostEqual(-pi / 2, angles[2], places=2, msg='got a wrong value for angle 3')
        self.assertAlmostEqual(-pi / 4, angles[3], places=2, msg='got a wrong value for angle 4')

    # expected robot pose:
    #     |\
    #     | \
    #     |  \_____
    #     |
    #     |
    # ----------- y-->
    def test3(self):
        a2 = 20
        d4 = 10
        d6 = 10
        config = TestConfig(0, a2, d4, d6)

        controller = ArmController(config)

        phi = 0
        x = 0
        y = d4 * cos(-pi / 4) + d6
        z = a2 + d4 * sin(-pi / 4)
        print(x, y, z)

        angles = controller.run(x, y, z, phi)
        self.assertAlmostEqual(pi / 2, angles[0], places=2, msg='got a wrong value for angle 1')
        self.assertAlmostEqual(pi / 2, angles[1], places=2, msg='got a wrong value for angle 2')
        self.assertAlmostEqual(-3*pi / 4, angles[2], places=2, msg='got a wrong value for angle 3')
        self.assertAlmostEqual(pi / 4, angles[3], places=2, msg='got a wrong value for angle 4')

    # expected robot pose:
    #     |\
    #     | \
    #     |  \
    #     |   |
    #     |   |
    # ----------- y-->
    def test4(self):
        a2 = 20
        d4 = 10
        d6 = 10
        config = TestConfig(0, a2, d4, d6)

        controller = ArmController(config)

        phi = -pi/2
        x = 0
        y = d4 * cos(-pi/4) + d6 * cos(phi)
        z = a2 + d4 * sin(-pi/4) + d6 * sin(phi)

        print(x, y, z)

        angles = controller.run(x, y, z, phi)
        self.assertAlmostEqual(pi / 2, angles[0], places=2, msg='got a wrong value for angle 1')
        self.assertAlmostEqual(pi / 2, angles[1], places=2, msg='got a wrong value for angle 2')
        self.assertAlmostEqual(-3 * pi / 4, angles[2], places=2, msg='got a wrong value for angle 3')
        self.assertAlmostEqual(-pi / 4, angles[3], places=2, msg='got a wrong value for angle 4')


class TestConfig:

    def __init__(self, d1, a2, d4, d6):
        self.d1 = d1
        self.a2 = a2
        self.d4 = d4
        self.d6 = d6
