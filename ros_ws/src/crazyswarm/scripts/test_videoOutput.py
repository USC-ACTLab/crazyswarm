"""Test video output in simulation.

This test is a bit strange: due to the following conditions

- Video capture runs until the script process stops,
- Video capture uses a child ffmpeg process to do compression,
- We register an `atexit` function to ensure that the pipe is flushed and
  ffmpeg process is closed cleanly,

we must run the video-generating script in a separate process.

Using `multiprocessing` would be easier, but `multiprocessing` processes don't
run `atexit` handlers when they exit. This design is controversial [1, 2].
Therefore, we must do a full `system()`-style process spawn witn `subprocess`
instead. To avoid adding another script just to support this test, this script
will behave as the video generator process when run as `__main__()`, and behave
as the test process when run via pytest.

[1] https://stackoverflow.com/questions/2546276/python-process-wont-call-atexit
[2] https://stackoverflow.com/questions/34506638/how-to-register-atexit-function-in-pythons-multiprocessing-subprocess
"""

import os
import subprocess
import sys

import numpy as np
import pytest

from pycrazyswarm import *

crazyflies_yaml = """
crazyflies:
- id: 1
  channel: 110
  initialPosition: [0.0, 0.0, 0.0]
"""

Z = 1.0
FPS = 12
DT = 1.0 / FPS
TOTAL_TIME = 4.0


def videoWriterProcess(path):
    args = "--sim --vis vispy --dt {} --video {}".format(DT, path)
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args=args)
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TOTAL_TIME / 2)
    timeHelper.sleep(TOTAL_TIME / 2)

    cf.goTo(cf.initialPosition + np.array([0.0, 1.0, Z]), yaw=0.0, duration=TOTAL_TIME / 2)
    timeHelper.sleep(TOTAL_TIME / 2)


@pytest.mark.skipif("TRAVIS" in os.environ or "CI" in os.environ,
                    reason="CI usually cannot create OpenGL context.")
def test_videoOutput(tmp_path):
    # tmp_path is supplied by pytest - a directory where we can write that will
    # eventually be deleted.
    path = str(tmp_path / "crazyswarm_test_video.mp4")
    subprocess.call(["python", __file__, path])

    import ffmpeg
    properties = ffmpeg.probe(path)
    stream = properties["streams"][0]
    file_duration = float(stream["duration"])
    file_fps = int(stream["avg_frame_rate"].split("/")[0])
    assert file_fps == FPS
    assert abs(file_duration - TOTAL_TIME) <= DT


if __name__ == "__main__":
    videoWriterProcess(sys.argv[1])
