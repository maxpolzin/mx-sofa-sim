from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor
# from stlib3.physics.rigid import Cube
from s90_servo import ServoMotor, ServoArm, ActuatedArm
from stlib3.scene import Scene

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
                  "Sofa.Component.ODESolver.Backward",
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
                  "Sofa.Component.LinearSolver.Iterative"]

    MainHeader(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], plugins=pluginList)
    ContactHeader(rootNode, alarmDistance=10, contactDistance=5, frictionCoef=0.8)

 



    noodleNode = rootNode.addChild('Noodle')

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




    # rootNode.addChild(ServoMotor(rootNode, scale3d=[100,100,100]))



    # rootNode.servomotor = rootNode.addChild(ServoMotor(name="ServoMotor"))
    # rootNode.servoarm = rootNode.servomotor.Articulation.ServoWheel.addChild(ServoArm(name="ServoArm"))
    # rootNode.servoarm.setRigidMapping(rootNode.ServoMotor.Articulation.ServoWheel.dofs.getLinkPath())

    # # # add a public attribute and connect it to the private one.
    # self.ServoMotor.angleIn.setParent(self.angleIn)

    # # add a public attribute and connect it to the internal one.
    # self.angleOut.setParent(self.ServoMotor.angleOut)


    scene = Scene(rootNode,iterative=False)
    arm = scene.Simulation.addChild(ActuatedArm(name='ActuatedArm', translation=[0.0, 0.0, 0.0]))

    

    floorNode = Floor(rootNode,
          translation=[0.0, -160.0, 0.0],
          rotation=[15.0, 0.0, 0.0],
          uniformScale=50.0,
          isAStaticObject=True)



    return rootNode
