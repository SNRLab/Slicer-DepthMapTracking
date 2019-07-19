 # -*- coding: utf-8 -*-

from __main__ import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
import os
import sys
import json
import math
import imageio
import numpy as np
from subprocess import check_output
import bpy #Requires moving the /2.7.9 directory to where the slicer python exe is in order to properly install

prot_no = 0

class DepthNetworkTracking:
    def __init__(self, parent):
        parent.title = "Depth Network Tracking"
        parent.categories = ["Utilities"]
        parent.contributors = ["Franklin King"]
        parent.helpText = """
        Depth Network Tracking
        """
        parent.acknowledgementText = """
        """
        #icon
        self.parent = parent


class DepthNetworkTrackingWidget:
    
    def __init__(self, parent=None):
      if not parent:
          self.parent = slicer.qMRMLWidget()
          self.parent.setLayout(qt.QVBoxLayout())
          self.parent.setMRMLScene(slicer.mrmlScene)
      else:
          self.parent = parent
          self.layout = self.parent.layout()
      if not parent:
          self.setup()
          self.parent.show()
      self.RGB_Paths = []
      self.Depth_Paths = []
      self.counter = 0
      self.surfaceRegistration = slicer.modules.surfaceregistration.widgetRepresentation().self()
 
    def setup(self):
      self.imagesDirectory = "D:/w/data/DepthNetworkData/5/"
      self.initializeButton = qt.QPushButton("Initialize")
      self.initializeButton.clicked.connect(self.Initialize)
      self.layout.addWidget(self.initializeButton)

      imagesCollapseButton = ctk.ctkCollapsibleButton()
      imagesCollapseButton.text = "Images"
      self.layout.addWidget(imagesCollapseButton)
      imagesLayout = qt.QHBoxLayout(imagesCollapseButton)

      self.depthMapImageView = qt.QLabel()
      self.RGBImageView = qt.QLabel()
      self.depthMapImageView.resize(200, 200)
      self.RGBImageView.resize(200, 200)
      imagesLayout.addWidget(self.depthMapImageView)
      imagesLayout.addWidget(self.RGBImageView)
      self.RGBImageView.setSizePolicy(qt.QSizePolicy.MinimumExpanding, qt.QSizePolicy.MinimumExpanding)
      self.depthMapImageView.setSizePolicy(qt.QSizePolicy.MinimumExpanding, qt.QSizePolicy.MinimumExpanding)

      self.imageNumber = qt.QLabel("-")
      self.layout.addWidget(self.imageNumber)

      controlsWidget = qt.QWidget()
      self.layout.addWidget(controlsWidget)
      forwardsBackwardsLayout = qt.QHBoxLayout(controlsWidget)

      self.backwardsButton = qt.QPushButton("<")
      self.backwardsButton.clicked.connect(self.Backwards)
      forwardsBackwardsLayout.addWidget(self.backwardsButton)      
      self.forwardsButton = qt.QPushButton(">")
      self.forwardsButton.clicked.connect(self.Forwards)
      forwardsBackwardsLayout.addWidget(self.forwardsButton)

      self.focalLengthBox = ctk.ctkDoubleSpinBox()
      self.focalLengthBox.maximum = 100000.0
      self.focalLengthBox.minimum = 0.0
      self.focalLengthBox.setValue(200.0)
      self.layout.addWidget(self.focalLengthBox)

      self.depthDividerBox = ctk.ctkDoubleSpinBox()
      self.depthDividerBox.setValue(2.5)
      self.depthDividerBox.maximum = 100000.0
      self.depthDividerBox.minimum = 0.0
      self.layout.addWidget(self.depthDividerBox)      

      self.pointCloudButton = qt.QPushButton("Create and render point cloud")
      self.pointCloudButton.clicked.connect(self.OnConvertToPointCloud)
      self.layout.addWidget(self.pointCloudButton)

      inputsWidget = qt.QWidget()
      self.layout.addWidget(inputsWidget)
      inputsLayout = qt.QFormLayout(inputsWidget)

      self.baseModelSelector = slicer.qMRMLNodeComboBox()
      self.baseModelSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
      self.baseModelSelector.selectNodeUponCreation = False
      self.baseModelSelector.addEnabled = False
      self.baseModelSelector.removeEnabled = True
      self.baseModelSelector.renameEnabled = True
      self.baseModelSelector.noneEnabled = False
      self.baseModelSelector.showHidden = False
      self.baseModelSelector.showChildNodeTypes = False
      self.baseModelSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Base Model: ", self.baseModelSelector)

      self.pointCloudSelector = slicer.qMRMLNodeComboBox()
      self.pointCloudSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
      self.pointCloudSelector.selectNodeUponCreation = True
      self.pointCloudSelector.addEnabled = True
      self.pointCloudSelector.removeEnabled = True
      self.pointCloudSelector.renameEnabled = True
      self.pointCloudSelector.noneEnabled = False
      self.pointCloudSelector.showHidden = False
      self.pointCloudSelector.showChildNodeTypes = False
      self.pointCloudSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Point Cloud Model: ", self.pointCloudSelector)

      self.booleanModelSelector = slicer.qMRMLNodeComboBox()
      self.booleanModelSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
      self.booleanModelSelector.selectNodeUponCreation = True
      self.booleanModelSelector.addEnabled = True
      self.booleanModelSelector.removeEnabled = True
      self.booleanModelSelector.renameEnabled = True
      self.booleanModelSelector.noneEnabled = False
      self.booleanModelSelector.showHidden = False
      self.booleanModelSelector.showChildNodeTypes = False
      self.booleanModelSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Boolean Model: ", self.booleanModelSelector)

      self.transientModelSelector = slicer.qMRMLNodeComboBox()
      self.transientModelSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
      self.transientModelSelector.selectNodeUponCreation = True
      self.transientModelSelector.addEnabled = True
      self.transientModelSelector.removeEnabled = True
      self.transientModelSelector.renameEnabled = True
      self.transientModelSelector.noneEnabled = False
      self.transientModelSelector.showHidden = False
      self.transientModelSelector.showChildNodeTypes = False
      self.transientModelSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Transient Model: ", self.transientModelSelector)            

      self.transientTransformSelector = slicer.qMRMLNodeComboBox()
      self.transientTransformSelector.nodeTypes = ( ("vtkMRMLLinearTransformNode"), "" )
      self.transientTransformSelector.selectNodeUponCreation = True
      self.transientTransformSelector.addEnabled = True
      self.transientTransformSelector.removeEnabled = True
      self.transientTransformSelector.renameEnabled = True
      self.transientTransformSelector.noneEnabled = False
      self.transientTransformSelector.showHidden = False
      self.transientTransformSelector.showChildNodeTypes = False
      self.transientTransformSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Transient Transform Node: ", self.transientTransformSelector)

      self.transformSelector = slicer.qMRMLNodeComboBox()
      self.transformSelector.nodeTypes = ( ("vtkMRMLLinearTransformNode"), "" )
      self.transformSelector.selectNodeUponCreation = True
      self.transformSelector.addEnabled = True
      self.transformSelector.removeEnabled = True
      self.transformSelector.renameEnabled = True
      self.transformSelector.noneEnabled = False
      self.transformSelector.showHidden = False
      self.transformSelector.showChildNodeTypes = False
      self.transformSelector.setMRMLScene(slicer.mrmlScene)
      inputsLayout.addRow("Transform Node: ", self.transformSelector)      

      self.iterationsSpinBox = qt.QSpinBox()
      self.iterationsSpinBox.setMaximum(50000)
      self.iterationsSpinBox.setMinimum(100)
      self.iterationsSpinBox.value = 1000
      inputsLayout.addRow("Iterations: ", self.iterationsSpinBox)

      self.landmarksSpinBox = qt.QSpinBox()
      self.landmarksSpinBox.setMaximum(50000)
      self.landmarksSpinBox.setMinimum(100)
      self.landmarksSpinBox.value = 1000
      inputsLayout.addRow("Landmarks: ", self.landmarksSpinBox)

      self.booleanCheckBox = qt.QCheckBox()
      self.booleanCheckBox.setChecked(False)
      inputsLayout.addRow("Use boolean: ", self.booleanCheckBox)


      self.registerButton = qt.QPushButton("Create, render, and register point cloud")
      self.registerButton.clicked.connect(self.OnRegister)
      self.layout.addWidget(self.registerButton)      

      spacer = qt.QSpacerItem(1, 14, qt.QSizePolicy.Expanding, qt.QSizePolicy.Expanding)
      #self.layout.addItem(spacer)      

    def Initialize(self):
      self.RGB_Paths = []
      self.Depth_Paths = []
      self.counter = 0

      for r,d,f in os.walk(self.imagesDirectory):
        for file in f:
          if file.startswith('brdf'):
            self.RGB_Paths.append(os.path.join(r,file))
          if file.startswith('depth'):
            self.Depth_Paths.append(os.path.join(r,file))
      self.UpdateImages()

    def Backwards(self):
      if (self.counter > 0):
        self.counter = self.counter-1
      self.UpdateImages()
      
    def Forwards(self):
      self.counter = self.counter+1
      self.UpdateImages()
      self.OnRegister()

    def OnConvertToPointCloud(self):
      pointCloudNode = self.DepthMapToPointCloud(self.Depth_Paths[self.counter], self.RGB_Paths[self.counter])

      # Permanently apply the transform to the point cloud first so that the ICP registration doesn't fall into a local minimum/maximum
      pointCloudNode.SetAndObserveTransformNodeID(self.transformSelector.currentNodeID)
      pointCloudNode.HardenTransform()

    def UpdateImages(self):
      pixmap = qt.QPixmap(self.RGB_Paths[self.counter])
      self.RGBImageView.setPixmap(pixmap.scaled(self.RGBImageView.size))
      pixmap = qt.QPixmap(self.Depth_Paths[self.counter])
      self.depthMapImageView.setPixmap(pixmap.scaled(self.depthMapImageView.size))
      self.imageNumber.setText(str(self.counter))
      
    def OnRegister(self):
      self.OnConvertToPointCloud()
      if self.booleanCheckBox.isChecked():
        self.IntersectBaseModel()
      self.RegisterToBaseModel()

    def IntersectBaseModel(self):
      transform = vtk.vtkTransform()
      matrix = vtk.vtkMatrix4x4()
      self.transformSelector.currentNode().GetMatrixTransformToParent(matrix)
      transform.SetMatrix(matrix)
      #transform.RotateX(-90.0)

      # cylinder = vtk.vtkCylinderSource()
      # cylinder.SetCenter(np.array([0.0, 0.0, 0.0]))
      # cylinder.SetRadius(25)
      # cylinder.SetHeight(60)
      # cylinder.SetResolution(100)
      # cylinder.Update()

      originalCylinder_path = os.path.dirname(os.path.realpath(__file__)) + "\\Models\\OriginalCylinder.stl"
      originalCylinderModelNode = slicer.util.loadModel(originalCylinder_path)        

      transformFilter = vtk.vtkTransformPolyDataFilter()
      transformFilter.SetInputData(originalCylinderModelNode.GetPolyData())
      transformFilter.SetTransform(transform)
      transformFilter.ReleaseDataFlagOn()
      transformFilter.Update()

      modelNode = self.booleanModelSelector.currentNode()
      modelNode.SetAndObservePolyData(transformFilter.GetOutput())
      modelDisplayNode = modelNode.GetModelDisplayNode()
      if modelDisplayNode is None:
        modelDisplayNode = slicer.vtkMRMLModelDisplayNode()
        modelDisplayNode.SetScene(slicer.mrmlScene)
        slicer.mrmlScene.AddNode(modelDisplayNode)
        modelNode.SetAndObserveDisplayNodeID(modelDisplayNode.GetID())
      modelDisplayNode.SetOpacity(0.2)
      modelDisplayNode.SetColor(0,1,0)

      modelNode2 = self.transientModelSelector.currentNode()
      modelDisplayNode2 = self.transientModelSelector.currentNode().GetModelDisplayNode()
      if modelDisplayNode2 is None:
        modelDisplayNode2 = slicer.vtkMRMLModelDisplayNode()
        modelDisplayNode2.SetScene(slicer.mrmlScene)
        slicer.mrmlScene.AddNode(modelDisplayNode2)
        modelNode2.SetAndObserveDisplayNodeID(modelDisplayNode2.GetID())
      modelDisplayNode2.SetOpacity(1.0)
      modelDisplayNode2.SetColor(0,1,0)      

      cylinder_path = os.path.dirname(os.path.realpath(__file__)) + "\\Models\\cylinder.stl"
      slicer.util.saveNode(modelNode, cylinder_path)

      slicer.mrmlScene.RemoveNode(originalCylinderModelNode)     
      
      self.blenderBoolean()

      
    def blenderBoolean(self):
      bpy.ops.wm.read_factory_settings()

      for scene in bpy.data.scenes:
          for obj in scene.objects:
              scene.objects.unlink(obj)

      # only worry about data in the startup scene
      for bpy_data_iter in (bpy.data.objects, bpy.data.meshes, bpy.data.lamps, bpy.data.cameras):
        for id_data in bpy_data_iter:
            bpy_data_iter.remove(id_data)
      
      dir_path = os.path.dirname(os.path.realpath(__file__)) + "\\Models\\"
      #dir_path = "D:\\w\\s\\DepthNetworkTracking\\Model\\"
      airwayPath = dir_path + "airwayNoWall.stl"
      cylinderPath = dir_path + "cylinder.stl"
      
      bpy.ops.import_mesh.stl(filepath=airwayPath)
      bpy.ops.import_mesh.stl(filepath=cylinderPath)

      objects = bpy.data.objects
      airwayNoWall = objects['airwayNoWall']
      cylinder = objects['Cylinder']

      airwayBoolean = airwayNoWall.modifiers.new(type="BOOLEAN", name="bool_7")
      airwayBoolean.object = cylinder
      airwayBoolean.operation = 'INTERSECT'
      cylinder.hide = True
      bpy.context.scene.objects.active = bpy.data.objects['airwayNoWall']
      bpy.ops.object.modifier_apply(apply_as='DATA', modifier=airwayBoolean.name)

      airwayNoWall.select = False
      cylinder.select = True
      bpy.ops.object.delete()
      airwayNoWall.select = True
      airwayIsolatedPath = dir_path + "airwayIsolated.stl"
      bpy.ops.export_mesh.stl(filepath=airwayIsolatedPath)

      outputModelPath = dir_path + "airwayIsolated.STL"
      outputModelNode = slicer.util.loadModel(outputModelPath)

      modelNode2 = self.transientModelSelector.currentNode()
      modelDisplayNode2 = self.transientModelSelector.currentNode().GetModelDisplayNode()
      if modelDisplayNode2 is None:
        modelDisplayNode2 = slicer.vtkMRMLModelDisplayNode()
        modelDisplayNode2.SetScene(slicer.mrmlScene)
        slicer.mrmlScene.AddNode(modelDisplayNode2)
        modelNode2.SetAndObserveDisplayNodeID(modelDisplayNode2.GetID())
      modelDisplayNode2.SetOpacity(0.7)
      modelDisplayNode2.SetColor(0,1,0)

      modelNode2.SetAndObservePolyData(outputModelNode.GetPolyData())
      slicer.mrmlScene.RemoveNode(outputModelNode)
      
      
    def RegisterToBaseModel(self):
      # ICP Registration
      if self.booleanCheckBox.isChecked():
        self.surfaceRegistration.inputFixedModelSelector.currentNodeID = self.transientModelSelector.currentNodeID
      else:
        self.surfaceRegistration.inputFixedModelSelector.currentNodeID = self.baseModelSelector.currentNodeID
      
      self.surfaceRegistration.inputMovingModelSelector.currentNodeID = self.pointCloudSelector.currentNodeID
      self.surfaceRegistration.outputTransformSelector.currentNodeID = self.transientTransformSelector.currentNodeID
      #self.surfaceRegistration.landmarkTransformTypeButtonsSimilarity.checked = True
      self.surfaceRegistration.numberOfIterations.setValue(self.iterationsSpinBox.value)
      self.surfaceRegistration.numberOfLandmarks.setValue(self.landmarksSpinBox.value)
      self.surfaceRegistration.onComputeButton()

      # Add transient transform to total transform
      transientMatrix = vtk.vtkMatrix4x4()
      matrix = vtk.vtkMatrix4x4()
      resultMatrix = vtk.vtkMatrix4x4()
      self.transformSelector.currentNode().GetMatrixTransformToParent(matrix)
      self.transientTransformSelector.currentNode().GetMatrixTransformToParent(transientMatrix)
      vtk.vtkMatrix4x4.Multiply4x4(transientMatrix, matrix, resultMatrix)
      self.transformSelector.currentNode().SetMatrixTransformToParent(resultMatrix)

    def DepthMapToPointCloud(self, depthmapFilename, rgbFilename):
      # Create point cloud
      depthImage = imageio.imread(depthmapFilename)
      rgbImage = imageio.imread(rgbFilename)

      height = len(depthImage)
      width = len(depthImage[0])

      minimumDepth = np.amin(depthImage)
      maximumDepth = np.amax(depthImage)
      print(maximumDepth)

      points = vtk.vtkPoints()

      fx_d = self.focalLengthBox.value
      fy_d = self.focalLengthBox.value

      for u in range(height):
        for v in range(width):
          vflipped = width - (v + 1)
          z = depthImage[u][vflipped]
          z = z[0] / self.depthDividerBox.value

          if z < 60:
          #if True:
            points.InsertNextPoint(np.array([z * (u - (height/2)) / fx_d, z * (v - (width/2)) / fy_d, z]))

      # Create junk polygons so that Slicer can actually display the point cloud
      polydata = vtk.vtkPolyData()
      polydata.SetPoints(points)
      poly = vtk.vtkPolygon()
      poly.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
      for n in range(points.GetNumberOfPoints()):
        poly.GetPointIds().SetId(n, n)
      polys = vtk.vtkCellArray()
      polys.InsertNextCell(poly)
      polydata.SetPolys(polys)

      # Display the point cloud
      modelNode = self.pointCloudSelector.currentNode()
      modelNode.SetAndObservePolyData(polydata)
      modelDisplayNode = modelNode.GetModelDisplayNode()
      if modelDisplayNode is None:
        modelDisplayNode = slicer.vtkMRMLModelDisplayNode()
        modelDisplayNode.SetScene(slicer.mrmlScene)
        slicer.mrmlScene.AddNode(modelDisplayNode)
        modelNode.SetAndObserveDisplayNodeID(modelDisplayNode.GetID())
      modelDisplayNode.SetRepresentation(modelDisplayNode.PointsRepresentation)
      modelDisplayNode.SetPointSize(4)
      modelDisplayNode.SetOpacity(0.2)
      modelDisplayNode.SetColor(1,0,0)

      return modelNode

    def polydataBoolean(self, polyData1, polyData2, operation, triangleFilter=False, loop=False, clean=True):
    # Subtract/add polyData2 from polyData1
      if (not polyData1) or (not polyData2):
        return polyData1
        
      booleanFilter = vtk.vtkBooleanOperationPolyDataFilter()
      if loop:
        booleanFilter = vtk.vtkLoopBooleanPolyDataFilter()
        
      if operation=="difference" or operation=="subtract":
        booleanFilter.SetOperationToDifference()
      elif operation=="union" or operation=="addition":
        booleanFilter.SetOperationToUnion()
      elif operation=="intersection":
        booleanFilter.SetOperationToIntersection()
      else:
        return None      
        
      if triangleFilter:
        triangleFilter1 = vtk.vtkTriangleFilter()
        triangleFilter1.SetInputData(polyData1)
        triangleFilter1.Update()
        
        triangleFilter2 = vtk.vtkTriangleFilter()
        triangleFilter2.SetInputData(polyData2)
        triangleFilter2.Update()

        booleanFilter.SetInputData(0, triangleFilter1.GetOutput())
        booleanFilter.SetInputData(1, triangleFilter2.GetOutput())
      else:
        booleanFilter.SetInputData(0, polyData1)
        booleanFilter.SetInputData(1, polyData2)
        
      booleanFilter.Update()
      
      if clean:
        cleanFilter = vtk.vtkCleanPolyData()
        cleanFilter.SetInputData(booleanFilter.GetOutput())
        cleanFilter.PointMergingOn()
        cleanFilter.Update()
        print("blah")
        return cleanFilter.GetOutput()
      else:
        return booleanFilter.GetOutput()

