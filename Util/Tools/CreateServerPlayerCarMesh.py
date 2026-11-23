# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Builds the server player's Ford Crown mesh: SK_FordCrown01 without the
front headrests, which otherwise fill the side screens of -server-surround
from the driver's seat. The player's car swaps to it ("mesh" in
Config/ServerPlayerMirrors.json); every other Crown keeps the original.

The headrests are separate pieces of the mesh (they share M_Details with the
rest of the cabin), found by their bounds. Run headless:

  UnrealEditor-Cmd.exe CarlaUnreal.uproject -run=pythonscript
      -script=<this file> -unattended -nop4 -nosplash -NullRHI
"""

from collections import defaultdict

import unreal

SOURCE = '/Game/Carla/Static/Car/4Wheeled/FordCrown01/SK_FordCrown01'
TARGET = '/Game/Carla/Static/Car/4Wheeled/FordCrown01/SK_FordCrown01_Player'

# Pieces to remove, by bounds in mesh space (cm): x, y, z ranges a piece must
# lie inside. Both front headrests.
REMOVE = [
    ((-12.0, -2.0), (-49.0, -29.0), (111.0, 129.0)),
    ((-12.0, -2.0), (29.0, 49.0), (111.0, 129.0)),
]


def as_list(result):
    """Geometry Script returns (mesh, out list, ...); finds the list."""
    candidates = result if isinstance(result, tuple) else (result,)
    for item in candidates:
        name = type(item).__name__
        if name == 'GeometryScriptVectorList':
            return list(unreal.GeometryScript_List.convert_vector_list_to_array(item))
        if name == 'GeometryScriptTriangleList':
            return list(unreal.GeometryScript_List.convert_triangle_list_to_array(item))
        if name == 'GeometryScriptIndexList':
            return list(unreal.GeometryScript_List.convert_index_list_to_array(item))
        if isinstance(item, (unreal.Array, list)):
            return list(item)
    raise RuntimeError('no list in ' + repr([type(x) for x in candidates]))


def pieces(positions, triangles):
    """Triangle ids of each connected piece, welding split vertices."""
    key_of = {}
    weld = [key_of.setdefault((round(p.x, 2), round(p.y, 2), round(p.z, 2)), len(key_of)) for p in positions]
    parent = list(range(len(key_of)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for t in triangles:
        a, b, c = (find(weld[i]) for i in (t.x, t.y, t.z))
        parent[b] = a
        parent[find(c)] = a
    groups = defaultdict(list)
    for index, t in enumerate(triangles):
        groups[find(weld[t.x])].append(index)
    return list(groups.values())


def inside(points, box):
    return all(box[0][0] <= p.x <= box[0][1] and box[1][0] <= p.y <= box[1][1] and box[2][0] <= p.z <= box[2][1]
               for p in points)


def main():
    assets = unreal.EditorAssetLibrary
    if assets.does_asset_exist(TARGET):
        assets.delete_asset(TARGET)
    target = assets.duplicate_asset(SOURCE, TARGET)
    if target is None:
        raise RuntimeError('cannot duplicate ' + SOURCE)

    mesh = unreal.DynamicMesh()
    options = unreal.GeometryScriptCopyMeshFromAssetOptions()
    lod = unreal.GeometryScriptMeshReadLOD()
    mesh, outcome = unreal.GeometryScript_AssetUtils.copy_mesh_from_skeletal_mesh(target, mesh, options, lod)
    if outcome != unreal.GeometryScriptOutcomePins.SUCCESS:
        raise RuntimeError('cannot read ' + TARGET)

    # The triangle ids may have gaps; positions are indexed by vertex id.
    positions = as_list(unreal.GeometryScript_MeshQueries.get_all_vertex_positions(mesh, False))
    triangles = as_list(unreal.GeometryScript_MeshQueries.get_all_triangle_indices(mesh, False))
    triangle_ids = as_list(unreal.GeometryScript_MeshQueries.get_all_triangle_i_ds(mesh))

    remove = []
    for piece in pieces(positions, triangles):
        points = [positions[i] for index in piece for i in (triangles[index].x, triangles[index].y, triangles[index].z)]
        for box in REMOVE:
            if inside(points, box):
                remove.extend(triangle_ids[index] for index in piece)
                unreal.log('CreateServerPlayerCarMesh: removing a piece of %d triangles in %s' % (len(piece), box))
    if not remove:
        raise RuntimeError('found nothing to remove; has the mesh changed?')

    index_list = unreal.GeometryScript_List.convert_array_to_index_list(remove, unreal.GeometryScriptIndexType.TRIANGLE)
    edit = next(getattr(unreal, name) for name in dir(unreal)
                if name.startswith('GeometryScript') and hasattr(getattr(unreal, name), 'delete_triangles_from_mesh'))
    mesh, deleted = edit.delete_triangles_from_mesh(mesh, index_list)
    unreal.log('CreateServerPlayerCarMesh: deleted %d triangles' % deleted)

    write = unreal.GeometryScriptCopyMeshToAssetOptions()
    target_lod = unreal.GeometryScriptMeshWriteLOD()
    mesh, outcome = unreal.GeometryScript_AssetUtils.copy_mesh_to_skeletal_mesh(mesh, target, write, target_lod)
    if outcome != unreal.GeometryScriptOutcomePins.SUCCESS:
        raise RuntimeError('cannot write ' + TARGET)
    assets.save_asset(TARGET, only_if_is_dirty=False)
    unreal.log('CreateServerPlayerCarMesh: saved ' + TARGET)


main()
