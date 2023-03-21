from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor
from stlib3.physics.rigid import Cube

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
                  "MultiThreading",
                  "Sofa.Component.Mapping.NonLinear",
                  "Sofa.Component.LinearSolver.Iterative"]

    MainHeader(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], plugins=pluginList)
    ContactHeader(rootNode, alarmDistance=10, contactDistance=5)



#     rootNode.addObject('GenericConstraintSolver', tolerance=1e-1, maxIterations=200)
#     rootNode.addObject('BruteForceBroadPhase')
#     rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint', responseParams='mu=0.6')




    rootNode.addObject('MeshOBJLoader', name='loader_noodle', triangulate=True, filename='Demos/mx_simulation/Body8_lowres_mm_2.obj')



    noodleNode = rootNode.addChild('Noodle')

    noodleNode.addObject('EulerImplicitSolver', name='cg_odesolver')
    noodleNode.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixd")


    noodleNode.addObject('SparseGridRamificationTopology', name='grid', n=[10, 10, 10], fileTopology="Demos/mx_simulation/Body8_lowres_mm_2.obj", nbVirtualFinerLevels="3", finestConnectivity="0")


    noodleNode.addObject('MechanicalObject', name='dofs', translation="0 1050 -50", rotation="120 -20 60")
    noodleNode.addObject('DiagonalMass', totalMass=2.0)


    noodleNode.addObject('HexahedronFEMForceField', name='fem', youngModulus="375",poissonRatio="0.07", method="large")
    noodleNode.addObject('GenericConstraintCorrection', name="constraint_correction")




    noodleVisu = noodleNode.addChild('noodleVisu')
    noodleVisu.addObject('OglModel', name='Visual', src='@../../loader_noodle', color="blue")
    noodleVisu.addObject('BarycentricMapping', input="@..", output="@Visual")



#     # collision
    noodleCollis = noodleNode.addChild('noodleCollis')


    noodleCollis.addObject('HexahedronSetTopologyContainer', src='@../../loader_noodle', name='container')



    noodleCollis.addObject('MechanicalObject', template='Vec3', name='dofs')
    noodleCollis.addObject('TriangleCollisionModel')
    noodleCollis.addObject('LineCollisionModel')
    noodleCollis.addObject('PointCollisionModel')
    noodleCollis.addObject('BarycentricMapping')




#     noodleCollis.addObject('MeshTopology', src='@../loader_noodle')
#     noodleCollis.addObject('MechanicalObject')
#     noodleCollis.addObject('TriangleCollisionModel')
#     noodleCollis.addObject('LineCollisionModel')
#     noodleCollis.addObject('PointCollisionModel')
#     noodleCollis.addObject('RigidMapping')
#     noodleCollis.addObject('QuadSetTopologyContainer', name="Container")
#     noodleCollis.addObject('QuadSetTopologyModifier', name="Modifier")
#     noodleCollis.addObject('QuadSetGeometryAlgorithms', template="Vec3d")
#     noodleCollis.addObject('QuadSetGeometryAlgorithms', input="@../Container", output="@Container")

#             #     <Hexa2QuadTopologicalMapping input="@../Container" output="@Container" />


      #   <Node name="Surf">
      #       <include href="Objects/HexahedronSetTopology.xml" src="@../grid" drawHexa="1" />
      #       <Node name="Q">
      #           <QuadSetTopologyContainer  name="Container" />
      #           <QuadSetTopologyModifier   name="Modifier" />
      #           <QuadSetGeometryAlgorithms name="GeomAlgo"   template="Vec3d" />
      #           <Hexa2QuadTopologicalMapping input="@../Container" output="@Container" />

      #           <TriangleCollisionModel />
      #           <PointCollisionModel />
      #       </Node>
      #   </Node>













    Floor(rootNode,
          translation=[0.0, -160.0, 0.0],
          uniformScale=50.0,
          isAStaticObject=True)

#     Floor(rootNode,
#           name="FloorObstacle",
#           translation=[0.0, -80.0, 0.0],
#           color=[0.0, 1.0, 0.0, 1.0],
#           uniformScale=0.8,
#           isAStaticObject=True)

#     for c in range(5):
#         cube = Cube(rootNode,
#                     name="Cube" + str(-210 + c * 70),
#                     translation=[-210 + c * 70, 0.0, 0.0],
#                     color=[c / 10.0, c * 0.7 / 10.0, 0.9, 1.0],
#                     uniformScale=20.0)
#         cube.addObject('UncoupledConstraintCorrection')

    return rootNode
