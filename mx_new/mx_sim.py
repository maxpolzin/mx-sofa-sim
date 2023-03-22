
import Sofa

from stlib3.scene import Scene, MainHeader, ContactHeader
from stlib3.solver import DefaultSolver

from stlib3.physics.rigid import Cube, Sphere, Floor
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel
from fixing_box import FixingBox

from stlib3.physics.mixedmaterial import Rigidify
from stlib3.components import addOrientedBoxRoi

from actuated_arm import ActuatedArm


class Noodlebot(Sofa.Prefab):
    """Creates an object composed of an elastic material."""
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]},
        {'name': 'poissonRatio', 'type': 'double', 'help': 'Poisson ratio', 'default': 0.3},
        {'name': 'youngModulus', 'type': 'double', 'help': "Young's modulus", 'default': 18000},
        {'name': 'totalMass', 'type': 'double', 'help': 'Total mass', 'default': 2.0}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):

        self.integration = self.addObject('EulerImplicitSolver', name='integration')
        self.solver = self.addObject('SparseLDLSolver', name="solver", template="CompressedRowSparseMatrixd")
        self.loader = self.addObject('MeshOBJLoader', name='loader', triangulate=True, filename='mesh/Body8_lowres_mm_2.obj', rotation=list(self.rotation.value), translation=list(self.translation.value),
                                         scale3d=list(self.scale.value))

        self.dofs = self.addObject('MechanicalObject', template='Vec3d', name='dofs')
        self.mass = self.addObject('DiagonalMass', totalMass=self.totalMass.value, name='mass')

        self.container = self.addObject('SparseGridRamificationTopology', n=[10, 10, 10], src=self.loader.getLinkPath(), name='container')



        self.forcefield = self.addObject('TetrahedronFEMForceField', template='Vec3d',
                                            method='large', name='forcefield',
                                            poissonRatio=self.poissonRatio.value, youngModulus=self.youngModulus.value)

        self.constraintcorrection = self.addObject('LinearSolverConstraintCorrection', name='constraintcorrection')


        self.visualmodel = self.addChild(VisualModel(visualMeshPath=self.loader.filename.value, rotation=list(self.rotation.value), translation=list(self.translation.value), scale=list(self.scale.value)))
        self.visualmodel.addObject('BarycentricMapping', name='mapping')


        self.collisionmodel = self.addChild('CollisionModel')
        self.collisionmodel.addObject('TriangleSetTopologyContainer', src='@../loader', name='container')
        self.collisionmodel.addObject('MechanicalObject', template='Vec3d', name='dofs')
        self.collisionmodel.addObject('TriangleCollisionModel')
        self.collisionmodel.addObject('LineCollisionModel')
        self.collisionmodel.addObject('PointCollisionModel')
        self.collisionmodel.addObject('BarycentricMapping')



 
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
                  "Sofa.Component.LinearSolver.Iterative",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.SolidMechanics.Spring"
                  ]



    scene = Scene(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], plugins=pluginList)
    scene.addMainHeader()

    scene.addObject('CollisionPipeline', name="DefaultPipeline") # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=150, contactDistance=30, frictionCoef=0.8)



    scene.Modelling.addChild(Noodlebot(translation=[0.0,500.0,0.0], rotation=[60,20,30], youngModulus=40))


    Cube(scene.Modelling, translation=[0.0,1200.0,0.0], uniformScale=100.1, color=[1.0, 0.0, 0.0, 1.0])
    scene.Modelling.Cube.addObject('UncoupledConstraintCorrection')


    Floor(scene.Modelling, translation=[0.0, -160.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=75.0, isAStaticObject=True)


    return rootNode
