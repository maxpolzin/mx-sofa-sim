import Sofa

from stlib3.scene import Scene
from stlib3.physics.rigid import Cube, Floor

from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.collision import CollisionMesh
from stlib3.components import addOrientedBoxRoi
from stlib3.physics.mixedmaterial import Rigidify

class CubeController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.name = "CubeController"
        return

    def reset(self):
        rotation = 0
        self.node.findData("angleIn").value = rotation

    def onAnimateBeginEvent(self, event):
        rotation = self.node.findData("angleIn").value
        rotation += 0.05
        self.node.findData("angleIn").value = rotation


class CubeBot(Sofa.Prefab):
    prefabParameters = []
    prefabData = [
        {
            "name": "angleIn",
            "help": "angle of rotation (in radians)",
            "type": "float",
            "default": 0,
        },
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)


   

    def init(self):


        cube_red = Cube(
            self,
            totalMass=50,
            name="Red",
            color=[1.0, 0.0, 0.0, 1.0],
            translation=[10.25, 3.0, 0.0],
            rotation=[90.0, 0.0, -90.0],
            uniformScale=0.5,
            isAStaticObject=False,
        )
        cube_red.addObject('UncoupledConstraintCorrection')

        angle = cube_red.addChild("Articulation")
        angle.addObject(
            "MechanicalObject",
            name="dofs",
            template="Vec1",
            position=[[self.getData("angleIn")]],
            rest_position=self.getData("angleIn").getLinkPath(),
        )
        angle.addObject("RestShapeSpringsForceField", points=0, stiffness=1e10)
        angle.addObject("ArticulatedHierarchyContainer", printLog=False)
        angle.addObject("UncoupledConstraintCorrection")

        articulationCenter = angle.addChild("ArticulationCenter")
        articulationCenter.addObject(
            "ArticulationCenter",
            posOnParent=[0, 0.0, 0.95],
            parentIndex=0,
            childIndex=0,
            posOnChild=[0.0, 0.0, 0.0],
        )

        articulation = articulationCenter.addChild("Articulations")
        articulation.addObject(
            "Articulation",
            translation=False,
            rotation=True,
            rotationAxis=[0, 0, 1],
            articulationIndex=0,
        )

        self.addObject(CubeController(node=self))

        cube_blue = Cube(
            angle,
            name="Blue",
            totalMass=50,
            color=[0.0, 0.0, 1.0, 1.0],
            uniformScale=0.5,
            isAStaticObject=False,
        )

        cube_blue.addObject(
            "ArticulatedSystemMapping",
            input1="@../dofs",
            input2="@../../mstate",
            output="@./",
        )




class NoodleRobot(Sofa.Prefab):

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.elasticMaterial = self.elasticBody(name="ElasticBody", translation=[19.5, 3.0, 0.0])
        self.ElasticBody.init()

        cubeBot = CubeBot()
        self.addChild(cubeBot)

        box = addOrientedBoxRoi(
            self,
            name="boxROIclamped",
            position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
            translation=[10.25, 3.0, 0.0],
            eulerRotation=[0.0, 0.0, 0.0],
            scale=[0.9, 1.1, 1.1],
        )
        box.drawBoxes = True
        box.init()

        indices = [[ind for ind in box.indices.value]]
        frame = [[0, 0, 0, 0, 0, 0, 1]]

        rigidifiedBody = Rigidify(
            self,
            self.elasticMaterial,
            groupIndices=indices,
            frames=frame,
            name="RigidifiedBody",
        )

        rigidifiedBody.DeformableParts.addObject('UncoupledConstraintCorrection')
        rigidifiedBody.RigidParts.RigidifiedParticules.addObject('UncoupledConstraintCorrection')

        rigidifiedBody.RigidParts.addObject(
            "RigidRigidMapping", index=0, input=self.CubeBot.Red.mstate.getLinkPath()
        )




        # self.elasticWheel = self.elasticBody(name="ElasticWheel",
        #                                      translation=[-3, 3.0, 2.2],
        #                                      rotation=[0.0, 245.0, 0.0],
        #                                      volumeMeshFileName="mesh/roach/60_wheel.msh",
        #                                      surfaceMeshFileName="mesh/roach/400_wheel.obj",
        #                                      collisionMesh="mesh/roach/60_wheel.stl"
        #                                      )
        # self.ElasticWheel.init()



        # wheel_box = addOrientedBoxRoi(
        #     self,
        #     name="wheelBoxROIclamped",
        #     position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
        #     translation=[10.25, 3.0, 0.0],
        #     eulerRotation=[0.0, 0.0, 0.0],
        #     scale=[0.9, 1.1, 1.1],
        # )
        # wheel_box.drawBoxes = True
        # wheel_box.init()

        # indices = [[ind for ind in wheel_box.indices.value]]
        # frame = [[0, 0, 0, 0, 0, 0, 1]]

        # rigidifiedStruct = Rigidify(
        #     self,
        #     self.elasticMaterial,
        #     groupIndices=indices,
        #     frames=frame,
        #     name="RigidifiedWheel",
        # )

        # rigidifiedStruct.DeformableParts.addObject('UncoupledConstraintCorrection')
        # rigidifiedStruct.RigidParts.RigidifiedParticules.addObject('UncoupledConstraintCorrection')







    def elasticBody(self, 
                    name="ElasticBody",
                    translation=[0.0, 0.0, 0.0], 
                    rotation=[0.0, 0.0, 0.0],
                    volumeMeshFileName="mesh/roach/300_torus.msh",
                    surfaceMeshFileName="mesh/roach/2400_torus.obj",
                    collisionMesh="mesh/roach/300_torus.stl"):
        
        body = self.addChild(name)
        e = body.addChild(
            ElasticMaterialObject(
                volumeMeshFileName=volumeMeshFileName,
                poissonRatio=0.3,
                youngModulus=10080000,
                totalMass=50.5,
                surfaceColor=[0.4, 1.0, 0.7, 1.0],
                surfaceMeshFileName=surfaceMeshFileName,
                translation=translation,
                rotation=rotation,
                scale=[0.015,0.015,0.015],
                collisionMesh=collisionMesh
            )
        )
        return e





def createScene(rootNode):
    """This is my first scene"""

    pluginList = [
        "Sofa.Component.AnimationLoop",
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
        "Sofa.Component.LinearSolver.Iterative",
    ]


    # Mesh generation from Fusion360: https://www.sofa-framework.org/community/forum/topic/easiest-way-from-stl-to-vtk/


    scene = Scene(
        rootNode,
        dt=0.01,
        gravity=[0.0, -9.81, 0.0],
        iterative=True,
        plugins=pluginList,
    )
    scene.addMainHeader()

    scene.addObject(
        "CollisionPipeline", name="DefaultPipeline"
    )  # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=1, contactDistance=0.03, frictionCoef=0.1)


    noodleRobot = NoodleRobot()
    scene.Simulation.addChild(noodleRobot)


    Floor(scene.Modelling, translation=[0.0, -3.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=0.40, isAStaticObject=True)

    return rootNode
