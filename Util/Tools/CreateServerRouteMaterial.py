# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Creates /Carla/Mission/M_ServerRoute (Plugins/Carla/Content/Mission), the
material of the server player's mission route line and waypoint markers: an
unlit, translucent, two-sided colour.

Run headless from a ue58-dev tree:

  UnrealEditor-Cmd.exe CarlaUnreal.uproject -run=pythonscript
      -script=<this file> -unattended -nop4 -nosplash -NullRHI

Parameters (set by FServerMission):
  Color       (vector)  the colour, green
  Brightness  (scalar)  multiplies it
  Opacity     (scalar)  0 to 1

The emissive output is divided by the viewer's exposure
(EyeAdaptationInverse), so the line looks the same in daylight and at night
instead of rendering nearly black in a physically lit map.
"""
import unreal

PATH = '/Carla/Mission'
NAME = 'M_ServerRoute'

mel = unreal.MaterialEditingLibrary
assets = unreal.EditorAssetLibrary
material = unreal.load_asset(PATH + '/' + NAME)
if material is not None:
    mel.delete_all_material_expressions(material)
else:
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        NAME, PATH, unreal.Material, unreal.MaterialFactoryNew())
material.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_UNLIT)
material.set_editor_property('blend_mode', unreal.BlendMode.BLEND_TRANSLUCENT)
material.set_editor_property('two_sided', True)


def node(cls, x, y):
    return mel.create_material_expression(material, cls, x, y)


color = node(unreal.MaterialExpressionVectorParameter, -800, 0)
color.set_editor_property('parameter_name', 'Color')
color.set_editor_property('default_value', unreal.LinearColor(0.05, 1.0, 0.15, 1.0))

brightness = node(unreal.MaterialExpressionScalarParameter, -800, 200)
brightness.set_editor_property('parameter_name', 'Brightness')
brightness.set_editor_property('default_value', 0.8)

opacity = node(unreal.MaterialExpressionScalarParameter, -800, 350)
opacity.set_editor_property('parameter_name', 'Opacity')
opacity.set_editor_property('default_value', 0.7)

rgb = node(unreal.MaterialExpressionComponentMask, -600, 0)
rgb.set_editor_property('r', True)
rgb.set_editor_property('g', True)
rgb.set_editor_property('b', True)
rgb.set_editor_property('a', False)
mel.connect_material_expressions(color, '', rgb, '')

bright = node(unreal.MaterialExpressionMultiply, -400, 0)
mel.connect_material_expressions(rgb, '', bright, 'A')
mel.connect_material_expressions(brightness, '', bright, 'B')

inverse = node(unreal.MaterialExpressionEyeAdaptationInverse, -400, 200)
exposed = node(unreal.MaterialExpressionMultiply, -200, 0)
mel.connect_material_expressions(bright, '', exposed, 'A')
mel.connect_material_expressions(inverse, '', exposed, 'B')

mel.connect_material_property(exposed, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR)
mel.connect_material_property(opacity, '', unreal.MaterialProperty.MP_OPACITY)

mel.recompile_material(material)
assets.save_asset(PATH + '/' + NAME, only_if_is_dirty=False)
unreal.log('created ' + PATH + '/' + NAME)
