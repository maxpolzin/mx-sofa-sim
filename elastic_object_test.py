
import Sofa

from stlib3.scene import Scene, MainHeader, ContactHeader
from stlib3.solver import DefaultSolver

from stlib3.physics.rigid import Cube, Sphere, Floor
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.collision import CollisionMesh

from stlib3.components import addOrientedBoxRoi
from stlib3.physics.mixedmaterial import Rigidify

from stlib3.visuals import VisualModel




class ServoMotor(Sofa.Prefab):
    """A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual modeli
    - a mechanical model composed two rigds. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab has the following properties:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def addScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    """
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale3d', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0e1, 1.0e1, 1.0e1]}]

    prefabData = [
        {'name': 'minAngle', 'help': 'min angle of rotation (in radians)', 'type': 'float', 'default': -100},
        {'name': 'maxAngle', 'help': 'max angle of rotation (in radians)', 'type': 'float', 'default': 100},
        {'name': 'angleIn', 'help': 'angle of rotation (in radians)', 'type': 'float', 'default': 0},
        {'name': 'angleOut', 'help': 'angle of rotation (in degree)', 'type': 'float', 'default': 0}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        # Servo body
        servoBody = self.addChild('ServoBody')
        servoBody.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.]],
                            translation=list(self.translation.value), rotation=list(self.rotation.value),
                            scale3d=list(self.scale3d.value))
        servoBody.addObject('FixedConstraint', indices=0)
        servoBody.addObject('UniformMass', totalMass=0.01)

        visual = servoBody.addChild('VisualModel')
        visual.addObject('MeshSTLLoader', name='loader', filename='mesh/SG90_servomotor_finger.stl', scale=1e1,
                         rotation=[0.0, -90.0, 0.0], translation=[-12.0e1, -5.0e1, 0.0])
        visual.addObject('MeshTopology', src='@loader')
        visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.addObject('RigidMapping', index=0)

        # Servo wheel
        angle = self.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[self.getData('angleIn')]],
                        rest_position=self.getData('angleIn').getLinkPath())
        angle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        angle.addObject('UniformMass', totalMass=10.0)

        servoWheel = angle.addChild('ServoWheel')
        servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                             position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]],
                             translation=list(self.translation.value), rotation=list(self.rotation.value),
                             scale3d=list(self.scale3d.value))
        servoWheel.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../ServoBody/dofs",
                             output="@./")

        articulationCenter = angle.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                     posOnChild=[0., 0., 0.])
        articulation = articulationCenter.addChild('Articulations')
        articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0],
                               articulationIndex=0)
        angle.addObject('ArticulatedHierarchyContainer', printLog=False)

        # The output
        self.angleOut.setParent(angle.dofs.position)



class ServoArm(Sofa.Prefab):
    """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

       Parameters:
            parent:        node where the ServoArm will be attached
            mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
            indexInput: (int) index of the rigid the ServoArm should be mapped to
    """

    prefabParameters = [
        {'name': 'mappingInputLink', 'type': 'string',
         'help': 'the rigid mechanical object that will control the orientation of the servo arm', 'default': ''},
        {'name': 'indexInput', 'type': 'int', 'help': 'index of the rigid the ServoArm should be mapped to',
         'default': 1}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject('MechanicalObject',
                       name='dofs',
                       size=1,
                       template='Rigid3',
                       showObject=False,
                       showObjectScale=5e1,
                       translation2=[0, 0, -25e1])

    def setRigidMapping(self, path):
        self.addObject('RigidRigidMapping', name='mapping', input=path, index=self.indexInput.value)

        visual = self.addChild(VisualModel(visualMeshPath='mesh/SG90_servoarm.stl', translation=[0, 0, 25.0e1],
                                           rotation=[-90, 0, 0],
                                           scale=[1.0e1, 1.0e1, 1.0e1], color=[1., 1., 1., 0.75]))
        visual.OglModel.writeZTransparent = True
        visual.addObject('RigidMapping', name='mapping')




class ActuatedWheel(Sofa.Prefab):
    """ActuatedWheel is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure

           Structure:
           Node : {
                name : 'ActuatedWheel'
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with its actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
    """
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0e-3, 1.0e-3, 1.0e-3]}]

    prefabData = [
        {'name': 'angleIn', 'group': 'ArmProperties', 'help': 'angle of rotation (in radians) of the arm',
         'type': 'float', 'default': 0},
        {'name': 'angleOut', 'group': 'ArmProperties', 'type': 'float', 'help': 'angle of rotation (in radians) of '
                                                                                'the arm', 'default': 0}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.servomotor = self.addChild(ServoMotor(name="ServoMotor", translation=self.translation.value,
                                                   rotation=self.rotation.value))
        self.servoarm = self.servomotor.Articulation.ServoWheel.addChild(ServoArm(name="ServoArm"))
        self.servoarm.setRigidMapping(self.ServoMotor.Articulation.ServoWheel.dofs.getLinkPath())

        # add a public attribute and connect it to the private one.
        self.ServoMotor.angleIn.setParent(self.angleIn)

        # add a public attribute and connect it to the internal one.
        self.angleOut.setParent(self.ServoMotor.angleOut)



class WheelController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs["node"]
        self.duration = 3.0
        self.time = 0.0
        self.objectDof = kwargs["objectDof"]
        self.actuator = kwargs["actuator"]
        self.forceContact = 0.0
        self.numContact = 0

        # Computation of the contact force applied on the object to grasp
        self.node.getRoot().GenericConstraintSolver.computeConstraintForces.value = True

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.P:
            print("Number of contact points: " + str(self.numContact))
            print("Norm of the contact force: " + str(self.forceContact))

    def onAnimateBeginEvent(self, eventType):

        # Update of the servomotor angular displacement
        # Rotation of pi/6 over self.duration (5s initially)
        angularStep = math.pi / 6
        angleInit = 0
        self.time += self.node.dt.value
        if self.time < self.duration:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep * self.time / self.duration
        else:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep

        # Computation of the contact force applied on the object to grasp
        contactForces = self.node.getRoot().GenericConstraintSolver.constraintForces.value

        # print the number of nodes in contact and the norm of the largest contact force
        self.numContact = 0
        self.forceContact = 0
        for contact in contactForces[0:-1:3]:
            if contact > 0:
                self.numContact += 1
                self.forceContact += contact
        self.forceContact /= self.node.dt.value




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


        # Load a servo motor
        wheel = self.addChild(ActuatedWheel(name="ActuatedWheel", rotation=[90.0, 0, 90.0], translation=[0, 0, 0]))
        wheel.ServoMotor.Articulation.dofs.position.value = [[wheel.angleIn.value]]  # Initialize the angle
        wheel.ServoMotor.minAngle.value = -2.02
        wheel.ServoMotor.maxAngle.value = -0.025


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


        servoArm = wheel.ServoMotor.Articulation.ServoWheel.ServoArm
        servoArm.addChild(rigidifiedStruct.RigidParts)
        servoArm.RigidParts.addObject('RigidRigidMapping', index=0, input=servoArm.dofs.getLinkPath())




        # # Add a fixing box to constrain the other part of the finger
        # FixingBox(self, self.elasticMaterial, translation=[10.0e-3, 0.0, 14.0e-3], scale=[15e-3, 25e-3, 6e-3])
        # self.FixingBox.BoxROI.drawBoxes = True

        self.addCollision()


    def elasticBody(self):
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
                  "ArticulatedSystemPlugin",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.LinearSolver.Iterative"]



    scene = Scene(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], iterative=True, plugins=pluginList)
    scene.addMainHeader()

    scene.addObject('CollisionPipeline', name="DefaultPipeline") # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=50, contactDistance=10, frictionCoef=0.8)


    noodleRobot = NoodleRobot()
    scene.Modelling.addChild(noodleRobot)

    


    # Add the simulated elements to the Simulation node
    scene.Simulation.addChild(noodleRobot.RigidifiedStructure.DeformableParts)
    scene.Simulation.DeformableParts.addObject('UncoupledConstraintCorrection')

    scene.Simulation.addChild(noodleRobot.RigidifiedStructure.RigidParts)
    scene.Simulation.RigidParts.addObject('UncoupledConstraintCorrection')

    scene.Simulation.addChild(noodleRobot.ActuatedWheel)


    Floor(scene.Modelling, translation=[0.0, -160.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=75.0, isAStaticObject=True)

    return rootNode
