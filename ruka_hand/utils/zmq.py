# Script for ZMQ utils
import base64
import pickle
import threading

import blosc as bl
import cv2
import numpy as np
import zmq


# Pub/Sub classes for Keypoints
class ZMQPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://{}:{}".format(self._host, self._port))

    def pub(self, data_array, topic_name):
        """
        Process the keypoints into a byte stream and input them in this function
        """
        buffer = pickle.dumps(data_array, protocol=-1)
        self.socket.send(bytes("{} ".format(topic_name), "utf-8") + buffer)

    def stop(self):
        print("Closing the publisher socket in {}:{}.".format(self._host, self._port))
        self.socket.close()
        self.context.term()


class ZMQSubscriber(threading.Thread):
    def __init__(self, host, port, topic):
        self._host, self._port, self._topic = host, port, topic
        self._init_subscriber()

        # Topic chars to remove
        self.strip_value = bytes("{} ".format(self._topic), "utf-8")

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect("tcp://{}:{}".format(self._host, self._port))
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self._topic, "utf-8"))

    def recv(self, flags=None):
        if flags is None:
            raw_data = self.socket.recv()
            raw_array = raw_data.lstrip(self.strip_value)
            # print(raw_array)
            return pickle.loads(raw_array)
        else:  # For possible usage of no blocking zmq subscriber
            try:
                raw_data = self.socket.recv(flags)
                raw_array = raw_data.lstrip(self.strip_value)
                return pickle.loads(raw_array)
            except zmq.Again:
                # print('zmq again error')
                return None

    def stop(self):
        print("Closing the subscriber socket in {}:{}.".format(self._host, self._port))
        self.socket.close()
        self.context.term()


class ZMQCameraSubscriber(threading.Thread):
    def __init__(self, host, port, topic_type):
        self._host, self._port, self._topic_type = host, port, topic_type
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        print("tcp://{}:{}".format(self._host, self._port))
        self.socket.connect("tcp://{}:{}".format(self._host, self._port))

        if self._topic_type == "Intrinsics":
            self.socket.setsockopt(zmq.SUBSCRIBE, b"intrinsics")
        elif self._topic_type == "RGB":
            self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")
        elif self._topic_type == "Depth":
            self.socket.setsockopt(zmq.SUBSCRIBE, b"depth_image")

    def recv_intrinsics(self):
        raw_data = self.socket.recv()
        raw_array = raw_data.lstrip(b"intrinsics ")
        return pickle.loads(raw_array)

    def recv_rgb_image(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
        encoded_data = np.fromstring(base64.b64decode(data["rgb_image"]), np.uint8)
        return cv2.imdecode(encoded_data, 1), data["timestamp"]

    def recv_depth_image(self):
        raw_data = self.socket.recv()
        striped_data = raw_data.lstrip(b"depth_image ")
        data = pickle.loads(striped_data)
        depth_image = bl.unpack_array(data["depth_image"])
        return np.array(depth_image, dtype=np.int16), data["timestamp"]

    def stop(self):
        print("Closing the subscriber socket in {}:{}.".format(self._host, self._port))
        self.socket.close()
        self.context.term()


class ZMQCameraPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        print("tcp://{}:{}".format(self._host, self._port))
        self.socket.bind("tcp://{}:{}".format(self._host, self._port))

    def pub_intrinsics(self, array):
        self.socket.send(b"intrinsics " + pickle.dumps(array, protocol=-1))

    def pub_rgb_image(self, rgb_image, timestamp):
        _, buffer = cv2.imencode(".jpg", rgb_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = dict(timestamp=timestamp, rgb_image=base64.b64encode(buffer))
        self.socket.send(b"rgb_image " + pickle.dumps(data, protocol=-1))

    def pub_depth_image(self, depth_image, timestamp):
        compressed_depth = bl.pack_array(
            depth_image, cname="zstd", clevel=1, shuffle=bl.NOSHUFFLE
        )
        data = dict(timestamp=timestamp, depth_image=compressed_depth)
        self.socket.send(b"depth_image " + pickle.dumps(data, protocol=-1))

    def stop(self):
        print("Closing the publisher socket in {}:{}.".format(self._host, self._port))
        self.socket.close()
        self.context.term()


def create_pull_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind("tcp://{}:{}".format(host, port))
    return socket
