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
  Warning     (0 to 1)   the blind-spot indicator's brightness, 0 hides it
  IconCenter  (vector)   where the indicator is, image texture coordinates (R, G)
  IconRadius  (scalar)   its radius, as a fraction of the image height
  Aspect      (scalar)   the image's width over its height, so the icon is round
  IconColor   (vector)   its colour, amber

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


def vector(name, default, x, y):
    expression = node(unreal.MaterialExpressionVectorParameter, x, y)
    expression.set_editor_property('parameter_name', name)
    expression.set_editor_property('default_value', unreal.LinearColor(*default))
    return expression


def binary(cls, a, b, x, y):
    expression = node(cls, x, y)
    mel.connect_material_expressions(a, '', expression, 'A')
    mel.connect_material_expressions(b, '', expression, 'B')
    return expression


def constant(value, x, y):
    expression = node(unreal.MaterialExpressionConstant, x, y)
    expression.set_editor_property('r', value)
    return expression


def saturate(source, x, y):
    expression = node(unreal.MaterialExpressionSaturate, x, y)
    mel.connect_material_expressions(source, '', expression, '')
    return expression


# The blind-spot indicator: an amber disc with a dark rim, faded in by Warning.
warning = scalar('Warning', 0.0, -800, 500)
center = mask(vector('IconCenter', (0.12, 0.2, 0.0, 0.0), -1200, 600), True, True, -1000, 600)
radius = scalar('IconRadius', 0.12, -800, 700)
aspect = append(scalar('Aspect', 1.5, -1200, 800), constant(1.0, -1200, 900), -1000, 800)
# Distances in image heights, so the disc is round on the glass.
distance = node(unreal.MaterialExpressionDistance, -600, 650)
mel.connect_material_expressions(binary(unreal.MaterialExpressionMultiply, uv, aspect, -800, 600), '', distance, 'A')
mel.connect_material_expressions(binary(unreal.MaterialExpressionMultiply, center, aspect, -800, 800), '', distance, 'B')
edge = binary(unreal.MaterialExpressionMultiply, radius, constant(0.15, -800, 900), -600, 850)
disc = saturate(binary(unreal.MaterialExpressionDivide,
                       binary(unreal.MaterialExpressionSubtract, radius, distance, -450, 650), edge, -300, 700), -150, 700)
rim_radius = binary(unreal.MaterialExpressionMultiply, radius, constant(1.3, -600, 1000), -450, 950)
rim = saturate(binary(unreal.MaterialExpressionDivide,
                      binary(unreal.MaterialExpressionSubtract, rim_radius, distance, -450, 800), edge, -300, 850), -150, 850)
dark = binary(unreal.MaterialExpressionMultiply, binary(unreal.MaterialExpressionMultiply, rim, warning, 0, 850),
              constant(0.7, 0, 950), 150, 850)
color = vector('IconColor', (1.0, 0.42, 0.02, 1.0), 0, 500)
amber = node(unreal.MaterialExpressionComponentMask, 300, 500)
amber.set_editor_property('r', True)
amber.set_editor_property('g', True)
amber.set_editor_property('b', True)
amber.set_editor_property('a', False)
mel.connect_material_expressions(color, '', amber, '')
image = lerp(multiply, constant(0.0, 150, 300), dark, 300, 200)
lit = binary(unreal.MaterialExpressionMultiply, disc, warning, 150, 700)
multiply = lerp(image, amber, lit, 450, 200)

# The capture is already exposed (its own auto exposure, like a camera), so
# undo the viewer's exposure: in a physically lit map an unlit emissive of 1
# would otherwise render nearly black.
inverse = node(unreal.MaterialExpressionEyeAdaptationInverse, 450, 400)
exposed = node(unreal.MaterialExpressionMultiply, 650, 200)
mel.connect_material_expressions(multiply, '', exposed, 'A')
mel.connect_material_expressions(inverse, '', exposed, 'B')
mel.connect_material_property(exposed, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR)

mel.recompile_material(material)
assets.save_asset(PATH + '/' + NAME, only_if_is_dirty=False)
unreal.log('created ' + PATH + '/' + NAME)
