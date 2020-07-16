import ffmpeg
import numpy as np
import os
import pytest

from pycrazyswarm import *


crazyflies_yaml = """
crazyflies:
- id: 1
  channel: 110
  initialPosition: [0.0, 0.0, 0.0]
"""

Z = 1.0


@pytest.mark.skipif("TRAVIS" in os.environ, reason="Travis-CI cannot create OpenGL context.")
def test_videoOutput(tmp_path):
    # tmp_path is supplied by pytest - a directory where we can write that will
    # eventually be deleted.
    fps = 12
    dt = 1.0 / fps
    args = "--sim --vis vispy_headless --dt {}".format(dt)
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args=args)
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    visualizer = timeHelper.visualizer

    path = str(tmp_path / "crazyswarm_test_video.mp4")
    visualizer.startVideoOutput(path, dt)

    cf.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(1.0)

    cf.goTo(cf.initialPosition + np.array([0.0, 1.0, Z]), yaw=0.0, duration=2.0)
    timeHelper.sleep(1.0)

    visualizer.finishVideoOutput()

    properties = ffmpeg.probe(path)
    stream = properties["streams"][0]
    file_duration = float(stream["duration"])
    file_fps = int(stream["avg_frame_rate"].split("/")[0])
    assert file_fps == fps
    assert abs(file_duration - timeHelper.time()) <= dt
