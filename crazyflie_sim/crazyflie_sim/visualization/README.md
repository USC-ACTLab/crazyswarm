# Visualizations

## Blender

### Dependencies

- `bpy`
- `rowan`
- `numpy` 
- `yaml` 

### Output
- Takes pictures from robots' perspectives
- Saves pictures in working directory in `simulation_results/<date-and-time>` with the following folder structure

    ```
    simulation_results
    └── <date-and-time>
        ├── cf1
        │   ├── calibration.yaml
        │   ├── cf0.csv
        │   ├── cf0_00000.jpg
        │   ├── cf0_00001.jpg
        │   └── ...
        ├── ...
        └── cfn
            ├── calibration.yaml
            ├── cfn.csv
            ├── cfn_00000.jpg
            ├── cfn_00001.jpg
            └── ...
    ```
    where `<name>.csv` contains the states in world coordinates of the camera or crazyflie, `calibration.yaml` contains the calibration information of the cameras and 
    `<name>_<frame>.jpg` is the `<frame>`th image taken from `<names>`'s perspective. If a robot is configured to not carry a camera, only `<name>.csv` will be recorded. 
- In order to use it, you need to add `blender` to the list of visualizations in `crazyflie/config/server.yaml` and run 

    ```sh
    ros2 launch crazyflie launch.py backend:=sim
    ```

### Configuration
- Crazyflies to appear in scene are defined in `crazyflie/config/crazyflies.yaml` 
- In `crazyflie/config/server.yaml` you **must** set the following parameters, unless stated otherwise:
    * `enabled: boolean`, enables or disables blender visualization in simulator
    * `fps: float`, frames per second rate of all cameras  
    * `auto_yaw: boolean`, enables or disables auto-yaw (optional, defaults to `false` if not set)
        - `radps: float`, radians per second
    * for every robot in `cf_cameras`:
        - `calibration`
            * `tvec: list[float]` translation vector of camera wrt. robot
            * `rvec: list[float]` rotation vector of camera wrt. robot (for camera +Z is front and +Y is up)  

            > **Note**
            > 1. The camera matrix $\mathbf K$ is set internally to be close to a real crazyflie's as possible.
            > It is given (in row-major order) by $$\mathbf  K = \left[ [170, 0, 160], [0, 170, 160], [0, 0, 1] \right].$$
            > 2. The distortion coefficient is set to $\left(0, 0, 0, 0, 0\right)^\top $

- Example configuration

    ```yaml
    blender:
      enabled: true
      fps: 1           # frames per second
      cycle_bg: false  # if true, pictures will cycle through different environemt background images (useful for synthetic image generation). Otherwise a single environment background image will be used
      cf_cameras:      # names of crazyflies with cameras on them if enabled in `crazyflies.yaml`
        cf231:
          calibration:
            tvec: [0,0,0]
            rvec: [1.2092,-1.2092,1.2092]   # 0 deg tilt
        cf5:
          calibration:
            tvec: [0,0,0]
            rvec: [ 0.61394313, -0.61394313,  1.48218982]   # 45 deg tilt
        cf6:
          calibration:
            tvec: [0,0,0]
            rvec: [0.0,0.0,1.5707963267948966]    # 90 deg tilt
    ```

- For efficiently generating synthetic images, we recommend setting the backend `none` in `crazyflie/config/server.yaml` 

### Acknowledgments 

- All background images are in the public domain (CC0 license) and were sourced from [polyhaven.com](https://polyhaven.com/) 
- The crazyflie model used is under an MIT license and was modified and sourced from [https://github.com/bitcraze/crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation)

