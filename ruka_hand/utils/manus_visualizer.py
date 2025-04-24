# Will listen to the stream manus data and plot them
import io
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from tqdm import tqdm

from ruka_hand.utils.constants import HOST, LEFT_STREAM_PORT, RIGHT_STREAM_PORT
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.video_recorder import VideoRecorder
from ruka_hand.utils.zmq import ZMQSubscriber


class ManusVisualizer:
    def __init__(
        self,
        host,
        frequency,
        hand_type,
        is_3d=False,
        save_dir=None,
        r2r_teleop=False,
    ):
        stream_port = LEFT_STREAM_PORT if hand_type == "left" else RIGHT_STREAM_PORT
        self.keypoints_subscriber = ZMQSubscriber(host, stream_port, "keypoints")
        self.timer = FrequencyTimer(frequency)
        self.hand_type = hand_type  # Change the direction according to the hand
        self.dir = 1 if r2r_teleop else -1
        self.is_3d = is_3d

        if not save_dir is None:
            self.video_recorder = VideoRecorder(
                save_dir=Path(save_dir),
                resize_and_transpose=True,
                render_size=480,
                fps=frequency,
            )
        self.save_dir = save_dir

    def _plot_line_online(self, X1, X2, Y1, Y2):
        plt.xlim(-12, 10)
        plt.ylim(0, 22)
        plt.plot([X1, X2], [Y1, Y2])

    def _plot_single_frame(self, keypoints):

        # Plot the thumb
        for finger_id in range(keypoints.shape[0]):

            # Plot the wrist
            if finger_id == 0:  # Wrist is included with the thumb
                self._plot_line_online(
                    self.dir * keypoints[0, 0, 0],  # 0,0 is the wrist
                    self.dir * keypoints[finger_id, 1, 0],
                    -keypoints[0, 0, 2],
                    -keypoints[finger_id, 1, 2],
                )
            else:
                self._plot_line_online(
                    self.dir * keypoints[0, 0, 0],  # 0,0 is the wrist
                    self.dir * keypoints[finger_id, 0, 0],
                    -keypoints[0, 0, 2],
                    -keypoints[finger_id, 0, 2],
                )

            knuckle_to_tip_range = (1, 4) if finger_id == 0 else (0, 4)
            # Drawing knuckle to knuckle connections and knuckle to fingertip connections
            for idx in range(knuckle_to_tip_range[0], knuckle_to_tip_range[1]):
                self._plot_line_online(
                    self.dir * keypoints[finger_id, idx, 0],
                    self.dir * keypoints[finger_id, idx + 1, 0],
                    -keypoints[finger_id, idx, 2],
                    -keypoints[finger_id, idx + 1, 2],
                )

        plt.draw()
        plt.savefig(
            f"single_frame_{self.hand_type}.png", bbox_inches="tight", format="png"
        )
        buf = io.BytesIO()
        plt.savefig(buf, bbox_inches="tight", format="png")
        buf.seek(0)
        image = np.array(Image.open(buf))
        if not self.save_dir is None:
            self.video_recorder.record(image)

        plt.cla()

    def _plot_line_online_3d(self, ax, x1, x2, y1, y2, z1, z2):
        ax.set_zlim(0, 22)
        ax.set_ylim(-10, 3)
        ax.set_xlim(-15, 15)
        ax.plot([x1, x2], [y1, y2], [z1, z2])

    def _plot_single_frame_3d(self, keypoints):

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        self._plot_thumb_bounds(ax)

        for finger_id in range(keypoints.shape[0]):

            # Plot the wrist
            if finger_id == 0:  # Wrist is included with the thumb
                self._plot_line_online_3d(
                    ax,
                    self.dir * keypoints[0, 0, 0],  # 0,0 is the wrist
                    self.dir * keypoints[finger_id, 1, 0],
                    keypoints[0, 0, 1],
                    keypoints[finger_id, 1, 1],
                    -keypoints[0, 0, 2],
                    -keypoints[finger_id, 1, 2],
                )

            else:
                self._plot_line_online_3d(
                    ax,
                    self.dir * keypoints[0, 0, 0],  # 0,0 is the wrist
                    self.dir * keypoints[finger_id, 0, 0],
                    keypoints[0, 0, 1],
                    keypoints[finger_id, 0, 1],
                    -keypoints[0, 0, 2],
                    -keypoints[finger_id, 0, 2],
                )

            knuckle_to_tip_range = (1, 4) if finger_id == 0 else (0, 4)
            # Drawing knuckle to knuckle connections and knuckle to fingertip connections
            for idx in range(knuckle_to_tip_range[0], knuckle_to_tip_range[1]):
                self._plot_line_online_3d(
                    ax,
                    self.dir * keypoints[finger_id, idx, 0],
                    self.dir * keypoints[finger_id, idx + 1, 0],
                    keypoints[finger_id, idx, 1],
                    keypoints[finger_id, idx + 1, 1],
                    -keypoints[finger_id, idx, 2],
                    -keypoints[finger_id, idx + 1, 2],
                )

        plt.draw()
        plt.savefig(
            f"single_frame_{self.hand_type}.png", bbox_inches="tight", format="png"
        )

        buf = io.BytesIO()
        plt.savefig(buf, bbox_inches="tight", format="png")
        buf.seek(0)
        image = np.array(Image.open(buf))
        if not self.save_dir is None:
            self.video_recorder.record(image)
        plt.cla()

    def stream(self, all_keypoints=None, video_name="hand_detections.mp4"):

        if all_keypoints is None:
            while True:
                self.timer.start_loop()
                try:
                    keypoints = self.keypoints_subscriber.recv()

                    if self.is_3d:
                        self._plot_single_frame_3d(keypoints)
                    else:
                        self._plot_single_frame(keypoints)

                    self.timer.end_loop()

                except KeyboardInterrupt:
                    break
        else:
            pbar = tqdm(total=len(all_keypoints))
            for keypoints in all_keypoints:
                if self.is_3d:
                    self._plot_single_frame_3d(keypoints)
                else:
                    self._plot_single_frame(keypoints)
                pbar.update(1)
                pbar.set_description("Visualizing offline keypoints")

            pbar.close()

        if not self.save_dir is None:
            self.video_recorder.save(video_name)


if __name__ == "__main__":

    viz = ManusVisualizer(
        host=HOST, hand_type="right", frequency=3, is_3d=True, is_robot=False
    )
    viz.stream()
