from pathlib import Path

import cv2
import imageio


class VideoRecorder:
    def __init__(self, save_dir, resize_and_transpose=True, render_size=256, fps=20):
        assert save_dir is not None, "Save Directory in VideoRecorder cannot be None"
        if isinstance(save_dir, str):
            save_dir = Path(save_dir)
        self.save_dir = save_dir
        self.save_dir.mkdir(exist_ok=True, parents=True)

        self.resize_and_transpose = resize_and_transpose
        self.render_size = render_size
        self.fps = fps
        self.frames = []

    def init(self, obs):
        self.frames = []
        self.record(obs)

    def record(self, obs):
        if self.resize_and_transpose:
            frame = cv2.resize(
                obs,
                dsize=(self.render_size, self.render_size),
                interpolation=cv2.INTER_CUBIC,
            )
        else:
            frame = obs
        self.frames.append(frame)

    def save(self, file_name):
        path = self.save_dir / file_name
        kargs = {"macro_block_size": None}
        imageio.mimsave(str(path), self.frames, fps=self.fps, **kargs)
