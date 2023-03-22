# -*- coding: utf-8 -*-
'''
Step 1:
We are introducing basic mechanical modeling, the new components bring
time integration and a mechanical object to the scene .
'''
import Sofa
from stlib3.scene import Scene


def Blueprint(name="Blueprint"):
    # Graphic modelling of the legends associated to the servomotors
    self = Sofa.Core.Node(name)
    self.addObject('MeshSTLLoader', name='loader', filename='blueprint.stl', scale3d=[10, 10, 10])
    self.addObject('OglModel', name='renderer', src='@loader')
    return self



def ElasticBody(name="ElasticBody", rotation=[0, 0, 0], translation=[0, 0, 0], color=[1.0, 1.0, 1.0, 1.0]):
    # To simulate an elastic object, we need:
    # - a deformation law (here linear elasticity)
    # - a solving method (here FEM)
    # - as we are using FEM we need a space discretization (here tetrahedron)
    self = Sofa.Core.Node(name)




    mechanicalmodel = self.addChild("MechanicalModel")

    mechanicalmodel.addObject('MeshOBJLoader',
                              name='loader',
                              rotation=rotation,
                              translation=translation,
                              triangulate=True,
                              filename='Body8_lowres_mm_2.obj')

    mechanicalmodel.addObject('SparseGridRamificationTopology', name='grid', n=[10, 10, 10], fileTopology="Demos/mx_simulation/Body8_lowres_mm_2.obj", nbVirtualFinerLevels="3", finestConnectivity="0")

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              template='Vec3d',
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=2.0)


    # ForceField components
    mechanicalmodel.addObject('HexahedronFEMForceField',
                              name="linearElasticBehavior",
                              youngModulus=250,
                              poissonRatio=0.45)

    mechanicalmodel.addObject('GenericConstraintCorrection', name="constraint_correction")



    mechanicalmodel.addObject('OglModel',
                          src='@loader',
                          name='renderer',
                          color=color)

    # Visual model
    # visualmodel = Sofa.Core.Node("VisualModel")

    # visualmodel = mechanicalmodel.addChild('noodleVisu')


    # Specific loader for the visual model


    # visualmodel.addObject('OglModel', name='Visual', src='@../loader', color="blue")
    # visualmodel.addObject('BarycentricMapping')


    # visualmodel.addObject('MeshOBJLoader',
    #                           name='loader',
    #                           rotation=rotation,
    #                           translation=translation,
    #                           triangulate=True,
    #                           filename='Body8_lowres_mm_2.obj')

    # visualmodel.addObject('OglModel',
    #                       src='@../loader',
    #                       name='renderer',
    #                       color=color)

    # # self.addChild(visualmodel)

    # visualmodel.addObject('RigidMapping')
    #                     #   input=mechanicalmodel.dofs.getLinkPath(),
    #                     #   output=visualmodel.renderer.getLinkPath())


                          
    return self









def createScene(rootNode):

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



    # Setting the gravity, assuming the length unit is in millimeters
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=pluginList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')

    # Setting the timestep in seconds
    rootNode.dt = 0.01

    scene.Modelling.addChild(Blueprint())

    scene.Modelling.addChild(ElasticBody(rotation=[90, 0, 0], color=[1.0, 1.0, 1.0, 0.5]))



    scene.Simulation.addChild(scene.Modelling.ElasticBody)


                             
    # scene.Simulation.addChild(elasticbody)                          
