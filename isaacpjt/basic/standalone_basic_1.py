from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


# World 생성
world = World(stage_units_in_meters=1.0)

# Stage 접근
stage = omni.usd.get_context().get_stage()


# 빨간 큐브 생성
cube_prim = DynamicCuboid(
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),   # 바닥에서 1m 높이
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),      # 빨강
)


# 바닥 + 큐브 추가
world.scene.add_default_ground_plane()
world.scene.add(cube_prim)

world.reset()


# 시뮬레이션
while simulation_app.is_running():
    world.step(render=True)


simulation_app.close()