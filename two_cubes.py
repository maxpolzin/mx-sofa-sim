import Sofa

from stlib3.scene import Scene
from stlib3.physics.rigid import Cube, Floor


class CubeController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.name = "CubeController"
        return

    def reset(self):
        rotation = 0
        self.node.Simulation.CubeBot.findData('angleIn').value = rotation

    def onAnimateBeginEvent(self, event):
        rotation = self.node.Simulation.CubeBot.findData('angleIn').value
        rotation += 0.05
        self.node.Simulation.CubeBot.findData('angleIn').value = rotation


class CubeBot(Sofa.Prefab):

    prefabParameters = []
    prefabData = [
        {'name': 'angleIn', 'help': 'angle of rotation (in radians)', 'type': 'float', 'default': 0},
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):

        cube_red = Cube(self, totalMass=500, name="Red", color=[1., 0., 0., 1.], translation=[1.25, 5.0, 0.0], rotation=[0.0, 90.0, 0.0], uniformScale=1.0, isAStaticObject=False)    

        angle = cube_red.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[self.getData('angleIn')]], rest_position=self.getData('angleIn').getLinkPath())
        angle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e7)


        articulationCenter = angle.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=0, posOnParent=[0, 0., 2.8], posOnChild=[0., 0., 0.])
        
        articulation = articulationCenter.addChild('Articulations')
        articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0],articulationIndex=0)
        
        angle.addObject('ArticulatedHierarchyContainer', printLog=False)


        cube_blue = Cube(angle, name="Blue", totalMass=500, color=[0., 0., 1., 1.], translation=[-0.75, 5.5, 0.0], rotation=[45.0, 0.0, 20.0], uniformScale=0.8, isAStaticObject=False)

        cube_blue.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../mstate", output="@./")




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

    cubeBot = CubeBot()
    scene.Simulation.addChild(cubeBot)

    rootNode.addObject(CubeController(node=rootNode))

    Floor(scene.Modelling, translation=[0.0, -3.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=0.25, isAStaticObject=True)

    return rootNode