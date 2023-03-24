
import Sofa

from stlib3.scene import Scene, MainHeader, ContactHeader
from stlib3.solver import DefaultSolver

from stlib3.physics.rigid import Cube, Sphere, Floor
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.collision import CollisionMesh

from stlib3.components import addOrientedBoxRoi
from stlib3.physics.mixedmaterial import Rigidify



class NoodleRobot(Sofa.Prefab):
    prefabParameters = [
        {"name": "rotation", "type": "Vec3d", "help": "Rotation in base frame", "default": [0.0, 0.0, 0.0]},
        {"name": "translation", "type": "Vec3d", "help": "Translation in base frame",
         "default": [0.0, 0.0, 0.0]}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    # Construct the actuated finger
    def init(self):
        # Load the finger mesh and create an elastic body from it
        self.elasticMaterial = self.elasticBody()
        self.ElasticBody.init()

        # Define a region of interest to rigidify the nodes of the finger mesh clamped in the servo arm
        box = addOrientedBoxRoi(self,
                                name="boxROIclamped",
                                position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
                                translation=[0.0, 320.0, 0.0],
                                eulerRotation=[0.0, 0.0, 0.0],
                                scale=[500, 300, 1000])
        box.drawBoxes = True
        box.init()

        # Get the indices of the finger mesh in the ROI, and the ROI frame
        indices = [[ind for ind in box.indices.value]]
        frame = [[0, 0, 0, 0, 0, 0, 1]]


        # Rigidify the finger nodes in the ROI. Create a Rigidified object and set up a spring force
        # field to constrain the nodes to stay in the rest shape
        rigidifiedStruct = Rigidify(self, self.elasticMaterial, groupIndices=indices, frames=frame,
                                    name="RigidifiedStructure")

        # servoArm = arm.ServoMotor.Articulation.ServoWheel.ServoArm
        # servoArm.addChild(rigidifiedStruct.RigidParts)
        # servoArm.RigidParts.addObject('RigidRigidMapping', index=0, input=servoArm.dofs.getLinkPath())

        # # Add a fixing box to constrain the other part of the finger
        # FixingBox(self, self.elasticMaterial, translation=[10.0e-3, 0.0, 14.0e-3], scale=[15e-3, 25e-3, 6e-3])
        # self.FixingBox.BoxROI.drawBoxes = True

        self.addCollision()


    def elasticBody(self):
        # Create a body as a child of the parent (the actuated finger)
        body = self.addChild("ElasticBody")

        e = body.addChild(ElasticMaterialObject(
                                        volumeMeshFileName="mesh/Body8_lowres_mm_gmsh.msh",
                                        poissonRatio=0.3,
                                        youngModulus=1800,
                                        totalMass=1.5,
                                        surfaceColor=[0.4, 1.0, 0.7, 1.0],
                                        surfaceMeshFileName="mesh/Body8_lowres_mm_2.obj",
                                        translation=[0.0, 500.0, 0.0], 
                                        rotation=[0.0, 0.0, 0.0]))
        return e


    def addCollision(self):
        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh1',
                                   surfaceMeshFileName="mesh/body8_lowres_mm_gmsh.stl",
                                   rotation=[0.0, 0.0, 0.0], translation=[0.0, 500.0, 0.0])




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
                  "Sofa.Component.LinearSolver.Iterative"]



    scene = Scene(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], plugins=pluginList)
    scene.addMainHeader()

    scene.addObject('CollisionPipeline', name="DefaultPipeline") # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=50, contactDistance=10, frictionCoef=0.8)



    noodleRobot = NoodleRobot()
    scene.Modelling.addChild(noodleRobot)



    Floor(scene.Simulation, translation=[0.0, -160.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=75.0, isAStaticObject=True)

    return rootNode
