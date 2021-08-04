from __future__ import print_function

import ffmpeg
import numpy as np


class VideoWriter:

    def __init__(self, path, dt, shape):
        """Initializes video output to a file.

        Args:
            path (str): Filesystem path to write output.
            dt (float): Time duration in seconds of frame, 1/fps.
            shape (tuple(int, int)): Height, width dimensions of video frames.
        """
        height, width = shape
        size_str = "{}x{}".format(width, height)
        # crf=0 option specifies libx264's lossless mode, which still gives
        # surprisingly small files.
        self.ffmpegProcess = (
            ffmpeg
            .input("pipe:", format="rawvideo", pix_fmt="rgb24", s=size_str, r=1.0/dt)
            .output(path, vcodec="libx264", crf=0)
            .overwrite_output()
            .run_async(pipe_stdin=True)
        )
        self.path = path
        self.shape = shape
        self.frames = 0

    def writeFrame(self, frame):
        """Appends a video frame to the file.

        Args:
            frame (np.ndarray): RGB image of shape `shape + (3,)`, where
                `shape` is the constructor arg.
        """
        assert len(frame.shape) == 3
        assert frame.shape[2] == 3
        if frame.shape[:2] != self.shape:
            msg = (
                "Rendered frame shape changed after video output started: "
                "from {} to {}.".format(self.shape, frame.shape[:2])
            )
            raise ValueError(msg)
        framebytes = frame.astype(np.uint8).tobytes()
        self.ffmpegProcess.stdin.write(framebytes)
        self.frames += 1

    def close(self):
        """Closes the video output and prints a status message to console."""
        self.ffmpegProcess.stdin.close()
        self.ffmpegProcess.wait()
        self.ffmpegProcess = None
        print("wrote {} frames to {}".format(self.frames, self.path))
        
