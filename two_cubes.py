import Sofa

from stlib3.scene import Scene
from stlib3.physics.rigid import Cube, Floor


def createScene(rootNode):
    """This is my first scene"""

    pluginList = ["Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mapping.Linear",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Topology.Container.Grid",
                  "Sofa.Component.Visual",
                  "Sofa.Component.Mapping.NonLinear",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Engine.Select",
                  "ArticulatedSystemPlugin",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.LinearSolver.Iterative"]



    scene = Scene(rootNode, dt=0.01, gravity=[0.0, -9.81, 0.0], iterative=False, plugins=pluginList)
    scene.addMainHeader()

    scene.addObject('CollisionPipeline', name="DefaultPipeline") # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=0.5, contactDistance=0.01, frictionCoef=0.1)

    scene.Simulation.addObject('GenericConstraintCorrection')



    cube_red = Cube(scene.Simulation, color=[1., 0., 0., 1.], translation=[1.25, 5.0, 0.0], rotation=[0.0, 0.0, 0.0], uniformScale=1.0, isAStaticObject=False)
    cube_blue = Cube(scene.Simulation, color=[0., 0., 1., 1.], translation=[-1.25, 5.0, 0.0], rotation=[0.0, 0.0, 0.0], uniformScale=1.0, isAStaticObject=False)


    Floor(scene.Modelling, translation=[0.0, -3.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=0.25, isAStaticObject=True)

    return rootNode