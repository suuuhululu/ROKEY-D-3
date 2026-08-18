from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()


# 빨간 큐브
red_cube = DynamicCuboid(
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([1.0, 1.0, 1.0]),
    scale=np.array([0.2, 0.2, 0.2]),
    color=np.array([1.0, 0.0, 0.0]),
)


# 초록 큐브
green_cube = DynamicCuboid(
    prim_path="/World/GreenCube",
    name="green_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.2, 0.2, 0.2]),
    color=np.array([0.0, 1.0, 0.0]),
)


# 파란 큐브
blue_cube = DynamicCuboid(
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([-1.0, -1.0, 1.0]),
    scale=np.array([0.2, 0.2, 0.2]),
    color=np.array([0.0, 0.0, 1.0]),
)


# Scene 추가
world.scene.add_default_ground_plane()

world.scene.add(red_cube)
world.scene.add(green_cube)
world.scene.add(blue_cube)

world.reset()


while simulation_app.is_running():
    world.step(render=True)


simulation_app.close()