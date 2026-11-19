"""Creates /Carla/Mirrors/M_ServerMirror (Plugins/Carla/Content/Mirrors), the material the server
player's mirrors use: an unlit surface showing a scene-capture render target.

Run headless from a ue58-dev tree:

  UnrealEditor-Cmd.exe CarlaUnreal.uproject -run=pythonscript
      -script=<this file> -unattended -nop4 -nosplash -NullRHI

Parameters (set per mirror by FServerMirrors from ServerPlayerMirrors.json):
  Mirror      (texture)  the capture's render target
  FlipU       (0 or 1)   mirror the image left to right (1 for a real mirror)
  FlipV       (0 or 1)   flip it top to bottom
  SwapUV      (0 or 1)   swap the panel's U and V, for a panel turned 90 degrees
  Brightness  (scalar)   multiplies the image, a little under 1 like real glass

The capture's own exposure is kept: the emissive output is divided by the
viewer's (EyeAdaptationInverse), so the mirror shows what the capture saw at
its true brightness in a physically lit map.
"""
import unreal

PATH = '/Carla/Mirrors'
NAME = 'M_ServerMirror'

mel = unreal.MaterialEditingLibrary
assets = unreal.EditorAssetLibrary
# Rebuild in place when it exists (load directly: a headless run's asset
# registry may not have scanned the plugin's content yet).
material = unreal.load_asset(PATH + '/' + NAME)
if material is not None:
    mel.delete_all_material_expressions(material)
else:
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        NAME, PATH, unreal.Material, unreal.MaterialFactoryNew())
material.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_UNLIT)


def node(cls, x, y):
    return mel.create_material_expression(material, cls, x, y)


def scalar(name, default, x, y):
    expression = node(unreal.MaterialExpressionScalarParameter, x, y)
    expression.set_editor_property('parameter_name', name)
    expression.set_editor_property('default_value', default)
    return expression


def lerp(a, b, alpha, x, y):
    expression = node(unreal.MaterialExpressionLinearInterpolate, x, y)
    mel.connect_material_expressions(a, '', expression, 'A')
    mel.connect_material_expressions(b, '', expression, 'B')
    mel.connect_material_expressions(alpha, '', expression, 'Alpha')
    return expression


def mask(source, r, g, x, y):
    expression = node(unreal.MaterialExpressionComponentMask, x, y)
    expression.set_editor_property('r', r)
    expression.set_editor_property('g', g)
    expression.set_editor_property('b', False)
    expression.set_editor_property('a', False)
    mel.connect_material_expressions(source, '', expression, '')
    return expression


def one_minus(source, x, y):
    expression = node(unreal.MaterialExpressionOneMinus, x, y)
    mel.connect_material_expressions(source, '', expression, '')
    return expression


def append(a, b, x, y):
    expression = node(unreal.MaterialExpressionAppendVector, x, y)
    mel.connect_material_expressions(a, '', expression, 'A')
    mel.connect_material_expressions(b, '', expression, 'B')
    return expression


coords = node(unreal.MaterialExpressionTextureCoordinate, -1400, 0)
flip_u = scalar('FlipU', 1.0, -1400, 200)
flip_v = scalar('FlipV', 0.0, -1400, 300)
swap = scalar('SwapUV', 0.0, -1400, 400)

u = mask(coords, True, False, -1200, -50)
v = mask(coords, False, True, -1200, 50)
# Swap first, then flip.
su = lerp(u, v, swap, -1000, -50)
sv = lerp(v, u, swap, -1000, 50)
fu = lerp(su, one_minus(su, -900, -150), flip_u, -800, -50)
fv = lerp(sv, one_minus(sv, -900, 150), flip_v, -800, 50)
uv = append(fu, fv, -600, 0)

texture = node(unreal.MaterialExpressionTextureSampleParameter2D, -400, 0)
texture.set_editor_property('parameter_name', 'Mirror')
texture.set_editor_property('texture', unreal.load_asset('/Engine/EngineResources/DefaultTexture'))
mel.connect_material_expressions(uv, '', texture, 'UVs')

brightness = scalar('Brightness', 0.9, -400, 250)
multiply = node(unreal.MaterialExpressionMultiply, -150, 0)
mel.connect_material_expressions(texture, 'RGB', multiply, 'A')
mel.connect_material_expressions(brightness, '', multiply, 'B')
# The capture is already exposed (its own auto exposure, like a camera), so
# undo the viewer's exposure: in a physically lit map an unlit emissive of 1
# would otherwise render nearly black.
inverse = node(unreal.MaterialExpressionEyeAdaptationInverse, -150, 200)
exposed = node(unreal.MaterialExpressionMultiply, 50, 0)
mel.connect_material_expressions(multiply, '', exposed, 'A')
mel.connect_material_expressions(inverse, '', exposed, 'B')
mel.connect_material_property(exposed, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR)

mel.recompile_material(material)
assets.save_asset(PATH + '/' + NAME, only_if_is_dirty=False)
unreal.log('created ' + PATH + '/' + NAME)
