
import Sofa

from stlib3.scene import Scene, MainHeader, ContactHeader
from stlib3.solver import DefaultSolver

from stlib3.physics.rigid import Cube, Sphere, Floor
from stlib3.physics.deformable import ElasticMaterialObject

from stlib3.components import addOrientedBoxRoi
from stlib3.physics.mixedmaterial import Rigidify


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


    # Define a region of interest to rigidify the nodes of the finger mesh clamped in the servo arm
    box = addOrientedBoxRoi(scene.Modelling,
                            name="boxROIclamped",
                            position=[0,0,0],
                            translation=[0.0, 320.0, 0.0],
                            eulerRotation=[0.0, 0.0, 0.0],
                            scale=[500, 300, 1000])
    box.drawBoxes = True
    box.init()

    # Get the indices of the finger mesh in the ROI, and the ROI frame
    indices = [[ind for ind in box.indices.value]]
    frame = [[0, 0, 0, 0, 0, 0, 1]]

    print(indices)


    Floor(scene.Simulation, translation=[0.0, -160.0, 0.0], rotation=[15.0, 0.0, 0.0], uniformScale=75.0, isAStaticObject=True)

    return rootNode
