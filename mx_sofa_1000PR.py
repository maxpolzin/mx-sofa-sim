import Sofa
import math

from splib3.numerics import sin, cos, to_radians


from splib3.objectmodel import setData

from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor
# from stlib3.physics.rigid import Cube
# from s90_servo import ServoMotor, ServoArm, ActuatedArm
from stlib3.scene import Scene
from splib3.animation import animate

from actuatedarm import ActuatedArm
from elasticbody import ElasticBody

from stlib3.physics.mixedmaterial import Rigidify

from stlib3.components import addOrientedBoxRoi

from splib3.numerics import vec3
from splib3.numerics.quat import Quat


def Tripod(name="Tripod", radius=60, numMotors=3, angleShift=180.0):
    def __getTransform(index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi * 360 / fnumstep
        angle2 = fi * 360 / fnumstep + angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist * sin(to_radians(angle2)), -1.35, dist * cos(to_radians(angle2))]
        return translation, eulerRotation

    def __rigidify(self, radius=60, numMotors=3, angleShift=180.0):
        deformableObject = self.ElasticBody.MechanicalModel
        self.ElasticBody.init()
        dist = radius
        numstep = numMotors
        groupIndices = []
        frames = []
        for i in range(0, numstep):
            translation, eulerRotation = __getTransform(i, numstep, angleShift, radius, dist)

            box = addOrientedBoxRoi(self, position=[list(i) for i in deformableObject.dofs.rest_position.value],
                                    name="BoxROI" + str(i),
                                    translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                                    eulerRotation=eulerRotation, scale=[45, 15, 30])

            box.drawBoxes = False
            box.init()
            groupIndices.append([ind for ind in box.indices.value])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + list(
                Quat.createFromEuler([0, float(i) * 360 / float(numstep), 0], inDegree=True)))

        # Rigidify the deformable part at extremity to attach arms
        rigidifiedstruct = Rigidify(self, deformableObject, groupIndices=groupIndices, frames=frames,
                                    name="RigidifiedStructure")

    self = Sofa.Core.Node(name)
    self.actuatedarms = []
    for i in range(0, numMotors):
        name = "ActuatedArm" + str(i)
        translation, eulerRotation = __getTransform(i, numMotors, angleShift, radius, radius)
        arm = ActuatedArm(name=name, translation=translation, rotation=eulerRotation)

        # Add limits to angle that correspond to limits on real robot
        arm.ServoMotor.minAngle = -2.0225
        arm.ServoMotor.maxAngle = -0.0255
        self.actuatedarms.append(arm)
        self.addChild(arm)

    self.addChild(ElasticBody(translation=[0.0, 30, 0.0], rotation=[90, 0, 0], color=[1.0, 1.0, 1.0, 0.5]))

    __rigidify(self, radius, numMotors, angleShift)
    return self 





def createScene(rootNode):
    """This is my first scene"""

    pluginList = ["Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mapping.Linear",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Topology.Container.Grid",
                  "Sofa.Component.Topology.Mapping",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GL.Component.Rendering2D",
                  "Sofa.GL.Component.Shader",
                  "ArticulatedSystemPlugin",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.Mapping.NonLinear",
                  "Sofa.GUI.Component",
                  "Sofa.Component.LinearSolver.Iterative"]


    scene = Scene(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], iterative=False, plugins=pluginList)
    scene.addMainHeader()

    ContactHeader(scene, alarmDistance=10, contactDistance=5, frictionCoef=0.8)

    scene.addObject('DefaultVisualManagerLoop')



    scene.Modelling.addObject('GenericConstraintCorrection')


    noodleNode = scene.Modelling.addChild('Noodle')

    noodleNode.addObject('EulerImplicitSolver', name='cg_odesolver')
    noodleNode.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixd")


    noodleNode.addObject('MeshOBJLoader', name='loader_noodle', triangulate=True, filename='Demos/mx_simulation/Body8_lowres_mm_2.obj')
    
    
    noodleNode.addObject('SparseGridRamificationTopology', name='grid', n=[10, 10, 10], fileTopology="Demos/mx_simulation/Body8_lowres_mm_2.obj", nbVirtualFinerLevels="3", finestConnectivity="0")

    noodleNode.addObject('MechanicalObject', name='dofs', translation="0 1050 -50", rotation="120 -20 60")
    noodleNode.addObject('DiagonalMass', totalMass=2.0)

    noodleNode.addObject('HexahedronFEMForceField', name='fem', youngModulus="275",poissonRatio="0.07", method="large")
    noodleNode.addObject('GenericConstraintCorrection', name="constraint_correction")

    noodleVisu = noodleNode.addChild('noodleVisu')
    noodleVisu.addObject('OglModel', name='Visual', src='@../loader_noodle', color="blue")
    noodleVisu.addObject('BarycentricMapping')

    noodleCollis = noodleNode.addChild('noodleCollis')
    noodleCollis.addObject('HexahedronSetTopologyContainer', src='@../grid', name='container')
    noodleCollis.addObject('MechanicalObject', template='Vec3d', name='dofs')
    noodleCollis.addObject('TriangleCollisionModel')
    noodleCollis.addObject('LineCollisionModel')
    noodleCollis.addObject('PointCollisionModel')
    noodleCollis.addObject('BarycentricMapping')


    tripod = scene.Modelling.addChild(Tripod())
    tripod.BoxROI0.drawBoxes = True
    tripod.BoxROI1.drawBoxes = True
    tripod.BoxROI2.drawBoxes = True


    # Use this to activate some rendering on the rigidified object ######################################
    setData(tripod.RigidifiedStructure.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
    setData(tripod.RigidifiedStructure.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
            drawMode=1, showColor=[1., 1., 0., 1.])
    setData(tripod.RigidifiedStructure.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
    #####################################################################################################

    scene.Simulation.addChild(tripod)


    scene.Simulation.addObject("EulerImplicitSolver")
    # scene.Simulation.addObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)
    scene.Modelling.addObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)



    # actuatedFinger = ServoMotor(scale3d=[10, 10, 10])
    # scene.Modelling.addChild(actuatedFinger)


    # Add animations
    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn.value = -factor * math.pi / 4
    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)


    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v23.12
    scene.Simulation.addObject('MechanicalMatrixMapper',
                               template='Vec3,Rigid3',
                               name="RigidAndDeformableCoupling",
                               object1=tripod.RigidifiedStructure.DeformableParts.dofs.getLinkPath(),
                               object2=tripod.RigidifiedStructure.RigidParts.dofs.getLinkPath(),
                               nodeToParse=tripod.RigidifiedStructure.DeformableParts.MechanicalModel.getLinkPath())



    floorNode = Floor(rootNode,
          translation=[0.0, -160.0, 0.0],
          rotation=[15.0, 0.0, 0.0],
          uniformScale=50.0,
          isAStaticObject=True)



    return rootNode
