from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()


# 큰 빨간 큐브
big_cube = DynamicCuboid(
    prim_path="/World/BigCube",
    name="big_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)


# 작은 초록 큐브
small_cube = DynamicCuboid(
    prim_path="/World/SmallCube",
    name="small_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([0.0, 1.0, 0.0]),
)


world.scene.add_default_ground_plane()

world.scene.add(big_cube)
world.scene.add(small_cube)

world.reset()


while simulation_app.is_running():
    world.step(render=True)


simulation_app.close()