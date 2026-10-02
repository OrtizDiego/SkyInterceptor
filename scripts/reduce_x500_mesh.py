#!/usr/bin/env python3
"""Shrink the PX4 x500 frame mesh so it is light enough to vendor.

The upstream NXP-HGD-CF.dae (PX4/PX4-gazebo-models, BSD-3-Clause) is ~22 MB
and ~180k triangles. Most of that is the landing-gear plastic, the flight
controller case and the screws. This script decimates only those untextured
material groups, leaves the carbon-fibre (textured) and small parts at full
detail, drops the Blender light, and rounds every float array to 1e-5.

Usage (outside the ROS workspace; needs `pip install pycollada trimesh
fast-simplification networkx numpy`):

    python3 scripts/reduce_x500_mesh.py <upstream>/NXP-HGD-CF.dae \
        interceptor_ws/src/interceptor_drone/meshes/x500/x500_frame.dae
"""

import sys

import collada
from collada.source import FloatSource, InputList
import fast_simplification
import numpy as np
import trimesh

# Fraction of triangles kept per decimated material group. Groups not listed
# here are kept as they are.
DECIMATE = {
    'LandingPlastic-mesh': 0.25,
    'FMUK66-mesh': 0.2,
    'Metal-mesh': 0.35,
}
SHARP_ANGLE = np.radians(35.0)


def decimated_geometry(doc, geom, ratio):
    prim = next(p for p in geom.primitives
                if isinstance(p, collada.triangleset.TriangleSet))
    tris = prim.vertex[prim.vertex_index].reshape(-1, 3)
    mesh = trimesh.Trimesh(tris, np.arange(len(tris)).reshape(-1, 3), process=True)
    points, faces = fast_simplification.simplify(mesh.vertices, mesh.faces, 1.0 - ratio)
    mesh = trimesh.graph.smooth_shade(trimesh.Trimesh(points, faces, process=True),
                                      angle=SHARP_ANGLE)
    gid = geom.id
    sources = [
        FloatSource(gid + '-pos', np.asarray(mesh.vertices), ('X', 'Y', 'Z')),
        FloatSource(gid + '-norm', np.asarray(mesh.vertex_normals), ('X', 'Y', 'Z')),
    ]
    inputs = InputList()
    inputs.addInput(0, 'VERTEX', '#' + gid + '-pos')
    inputs.addInput(0, 'NORMAL', '#' + gid + '-norm')
    new = collada.geometry.Geometry(doc, gid, geom.name, sources)
    new.primitives.append(
        new.createTriangleSet(mesh.faces.astype(np.int32).flatten(), inputs, prim.material))
    print('%-28s %7d -> %6d triangles' % (gid, len(prim), len(mesh.faces)))
    return new


def round_sources(geom):
    for src in geom.sourceById.values():
        if isinstance(src, FloatSource):
            src.data = np.round(src.data, 5)
            src.save()


def main(src, dst):
    doc = collada.Collada(src)
    geoms = []
    for geom in doc.geometries:
        if geom.id in DECIMATE:
            geom = decimated_geometry(doc, geom, DECIMATE[geom.id])
        else:
            print('%-28s %7d    kept' % (geom.id, sum(len(p) for p in geom.primitives)))
        round_sources(geom)
        geoms.append(geom)
    by_id = {g.id: g for g in geoms}

    doc.geometries.clear()
    doc.geometries.extend(geoms)
    for node in doc.scene.nodes:
        for child in node.children:
            if isinstance(child, collada.scene.GeometryNode):
                child.geometry = by_id[child.geometry.id]
    doc.scene.nodes[:] = [
        n for n in doc.scene.nodes
        if not any(isinstance(c, collada.scene.LightNode) for c in n.children)]
    doc.lights.clear()
    doc.save()
    doc.write(dst)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
