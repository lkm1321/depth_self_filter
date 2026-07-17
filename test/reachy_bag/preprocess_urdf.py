#!/usr/bin/env python3
"""Rewrite a Reachy URDF so its mesh references resolve to a flat local dir.

The recorded reachy2 URDF mixes absolute /home/reachy/... paths, unexpanded
$(find ...) xacro, and package:// URIs across several description packages. For
an offline test we only have reachy_description + dynamixel_description meshes
on disk, so we remap every mesh by *basename* to a staging dir and rewrite the
filename to a container-visible path.

Usage:
  preprocess_urdf.py IN.urdf OUT.urdf STAGE_DIR CONTAINER_MESH_DIR
"""
import os
import re
import sys

# Meshes the recorded URDF names but that aren't shipped standalone; substitute
# the closest available collider.
ALIAS = {"torso_collider_no_cone.dae": "torso_collider.dae"}


def main():
    in_urdf, out_urdf, stage, cont = sys.argv[1:5]
    have = set(os.listdir(stage))
    resolved, missing = set(), set()

    def repl(m):
        base = os.path.basename(m.group(1))
        base = ALIAS.get(base, base)
        if base in have:
            resolved.add(base)
            return 'filename="file://%s/%s"' % (cont.rstrip("/"), base)
        missing.add(os.path.basename(m.group(1)))
        return m.group(0)

    out = re.sub(r'filename="([^"]+\.dae)"', repl, open(in_urdf).read())
    open(out_urdf, "w").write(out)
    print("resolved %d meshes, %d unresolved: %s"
          % (len(resolved), len(missing), sorted(missing)))


if __name__ == "__main__":
    main()
