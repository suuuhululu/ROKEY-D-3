from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)


world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()


#-------------------------------------------
#루프 제어 체험


step_count = 0

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1
    if step_count % 100 == 0:
        print("step:", step_count)

    if step_count == 500:
        simulation_app.close()

    if step_count == 300:
        cube_prim = DynamicCuboid(                              # 4. Prim
            prim_path="/World/RedCube",
            name="red_cube",
            position=np.array([0.0, 0.0, 1.0]),
            scale=np.array([0.3, 0.3, 0.3]),
            color=np.array([1.0, 0.0, 0.0]),
)

