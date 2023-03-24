
import Sofa

from stlib3.scene import Scene, MainHeader, ContactHeader
from stlib3.solver import DefaultSolver

from stlib3.physics.rigid import Cube, Sphere, Floor
from stlib3.physics.deformable import ElasticMaterialObject



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
                  "Sofa.Component.LinearSolver.Iterative"]



    scene = Scene(rootNode, dt=0.005, gravity=[0.0, -9810.0, 0.0], plugins=pluginList)
    scene.addMainHeader()

    scene.addObject('CollisionPipeline', name="DefaultPipeline") # To surpress warning from contactheader.py
    scene.addContact(alarmDistance=50, contactDistance=10, frictionCoef=0.8)


    # elasticobject = ElasticMaterialObject(volumeMeshFileName="mesh/Body8_lowres_mm_gmsh.msh", name="ElasticMaterialObject")

    elasticobject = ElasticMaterialObject(
                                    volumeMeshFileName="mesh/Body8_lowres_mm_gmsh.msh",
                                    collisionMesh="mesh/body8_lowres_mm_gmsh.stl",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.5,
                                    surfaceColor=[0.4, 1.0, 0.7, 1.0],
                                    surfaceMeshFileName="mesh/Body8_lowres_mm_2.obj",
                                    translation=[0.0, 500.0, 0.0], 
                                    rotation=[0.0, 0.0, 0.0])



    scene.Modelling.addChild(elasticobject)

    Floor(scene.Simulation, translation=[0.0, -160.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=75.0, isAStaticObject=True)

    return rootNode
