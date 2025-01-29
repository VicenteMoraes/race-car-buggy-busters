"""
This script generates a track using the track gen function
and spawns it using the standard cone model into gazebo
"""
import os
from pathlib import Path
from time import sleep
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from avai_lab.gazebo.service import remove_entity, spawn_entities
from avai_lab.gazebo.msg import Entity, EntityFactory, EntityFactory_V, EntityType, Pose, Vector3d
from avai_lab.track_gen import generate_track

WORLD = "empty"
cone_model = Path(os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")) / "cone.sdf"
assert cone_model.exists(), "Cone model does not exist"

def main():
    inner_track, _, outer_track = generate_track(30, size_x=5, size_y=5, refinements=2, n_points=40, track_width=5)
    cones = []
    cone_positions = np.concatenate((inner_track, outer_track))
    for idx, pos in enumerate(cone_positions):
        x, y = pos
        entity_factory = EntityFactory(sdf_filename=str(cone_model), name=f"{idx}", 
                                       pose=Pose(position=Vector3d(x=x, y=y)))
        cones.append(entity_factory)
    entity_factory_v = EntityFactory_V(data=cones)
    spawn_entities(WORLD, entity_factory_v)

    return
    # This part is to incrementally remove the cones
    entities = [Entity(name=str(i), type=EntityType.MODEL) for i in range(len(cone_positions))]
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = [executor.submit(remove_entity, WORLD, entity) for entity in entities]

if __name__ == "__main__":
    main()
