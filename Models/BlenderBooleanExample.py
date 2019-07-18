
# Only needs to be imported once
import bpy

# Save model from slicer
dir_path = os.path.dirname(os.path.realpath(__file__))
model1_path = dir_path + "\\model1.stl"
model2_path = dir_path + "\\model2.stl"
slicer.util.saveNode(model1Node, model1_path)
slicer.util.saveNode(model2Node, model2_path)

# Clear and initialize Blender scene
bpy.ops.wm.read_factory_settings()

for scene in bpy.data.scenes:
    for obj in scene.objects:
        scene.objects.unlink(obj)

for bpy_data_iter in (bpy.data.objects, bpy.data.meshes, bpy.data.lamps, bpy.data.cameras):
  for id_data in bpy_data_iter:
      bpy_data_iter.remove(id_data)

# Import models to Blender
bpy.ops.import_mesh.stl(filepath=model1_path)
bpy.ops.import_mesh.stl(filepath=model2_path)

objects = bpy.data.objects
model1 = objects['model1'] # Dependent on model name
model2 = objects['model2']

# Perform boolean operation
booleanOperation = model1.modifiers.new(type="BOOLEAN", name="bool_7")
booleanOperation.object = model2
booleanOperation.operation = 'INTERSECT'
model2.hide = True
bpy.context.scene.objects.active = bpy.data.objects['model1']
bpy.ops.object.modifier_apply(apply_as='DATA', modifier=booleanOperation.name)

# Delete other models and save desired output
model1.select = False
model2.select = True
bpy.ops.object.delete()
model1.select = True
modelOutput_path = dir_path + "\\modelOutput.stl"
bpy.ops.export_mesh.stl(filepath=modelOutput_path)

# Import back into Slicer
outputModelPath = dir_path + "\\modelOutput.stl"
outputModelNode = slicer.util.loadModel(outputModelPath)



