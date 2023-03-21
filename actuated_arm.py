# -*- coding: utf-8 -*-
""" ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
"""
import Sofa
from stlib3.visuals import VisualModel

from s90_servo import ServoMotor, ServoArm, ActuatedArm




def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math

    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    arm = scene.Simulation.addChild(ActuatedArm(name='ActuatedArm', translation=[0.0, 0.0, 0.0]))

    def myanimate(target, factor):
        target.angleIn.value = math.sin(factor * 2 * math.pi)

    animate(myanimate, {'target': arm}, duration=2., mode='loop')
