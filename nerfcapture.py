# -*- coding: utf-8 -*-
"""
Receiver for the free, open-source NeRFCapture iOS app (https://github.com/jc211/NeRFCapture).
The app publishes RGB + LiDAR depth frames over Wi-Fi with CycloneDDS.
Message definition follows instant-ngp/scripts/nerfcapture2nerf.py.
"""

import argparse
import time
from dataclasses import dataclass
from threading import Lock, Thread

import numpy as np
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from cyclonedds.core import Qos, Policy
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration


@dataclass
@annotate.final
@annotate.autoid("sequential")
class NeRFCaptureFrame(idl.IdlStruct, typename="NeRFCaptureData.NeRFCaptureFrame"):
    id: types.uint32
    annotate.key("id")
    timestamp: types.float64
    fl_x: types.float32
    fl_y: types.float32
    cx: types.float32
    cy: types.float32
    transform_matrix: types.array[types.float32, 16]
    width: types.uint32
    height: types.uint32
    image: types.sequence[types.uint8]
    has_depth: bool
    depth_width: types.uint32
    depth_height: types.uint32
    depth_scale: types.float32
    depth_image: types.sequence[types.uint8]


# frames are a few MB each; the default socket buffer drops them
DDS_CONFIG = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        {general}
        {discovery}
        <Internal>
            <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
"""
# pin the network interface when the PC has several (e.g. Wi-Fi + VPN)
DDS_INTERFACE = '<General><Interfaces><NetworkInterface address="{}"/></Interfaces></General>'
# unicast discovery for routers that block multicast between Wi-Fi clients.
# 7400 is the discovery port the app listens on.
DDS_PEER = '<Discovery><Peers><Peer address="{0}"/><Peer address="{0}:7400"/></Peers></Discovery>'


@dataclass
class Intrinsics:
    fx: float
    fy: float
    tx: float
    ty: float


class NeRFCaptureStream:
    """
    Mimics the parts of record3d.Record3DStream used by ipad_stream.py.
    """
    def __init__(self, iface=None, peer=None):
        self.iface = iface
        self.peer = peer
        self.on_new_frame = lambda: None
        self.on_stream_stopped = lambda: None
        self._frame = None
        self._lock = Lock()

    def connect(self):
        general = DDS_INTERFACE.format(self.iface) if self.iface else ''
        discovery = DDS_PEER.format(self.peer) if self.peer else ''
        self._domain = Domain(domain_id=0, config=DDS_CONFIG.format(general=general, discovery=discovery))
        participant = DomainParticipant()
        qos = Qos(Policy.Reliability.Reliable(max_blocking_time=duration(seconds=1)))
        topic = Topic(participant, "Frames", NeRFCaptureFrame, qos=qos)
        self._reader = DataReader(participant, topic)
        Thread(target=self._receive_loop, daemon=True).start()
        print('Waiting for frames from NeRFCapture (set the app to Online mode)')

    def _receive_loop(self):
        while True:
            sample = self._reader.read_next()
            if sample is None:
                time.sleep(0.001)
                continue
            if not sample.has_depth:
                print('Received a frame without depth. NeRFCapture only sends depth on LiDAR devices.')
                continue
            with self._lock:
                self._frame = sample
            self.on_new_frame()

    def _latest(self):
        with self._lock:
            return self._frame

    def get_depth_frame(self):
        f = self._latest()
        # float32 depth in meters, same as record3d
        return np.asarray(f.depth_image, dtype=np.uint8).view(np.float32) \
                 .reshape((f.depth_height, f.depth_width)).copy()

    def get_rgb_frame(self):
        f = self._latest()
        return np.asarray(f.image, dtype=np.uint8).reshape((f.height, f.width, 3)).copy()

    def get_intrinsic_mat(self):
        """
        Intrinsics scaled to the depth map resolution.
        """
        f = self._latest()
        s = f.depth_width / f.width
        return Intrinsics(f.fl_x * s, f.fl_y * s, f.cx * s, f.cy * s)


if __name__ == '__main__':
    # connection check without open3d: python nerfcapture.py [--iface <PC IP address>] [--peer <device IP address>]
    parser = argparse.ArgumentParser()
    parser.add_argument('--iface', help='IP address of the network interface connected to the device')
    parser.add_argument('--peer', help='IP address of the device, when multicast discovery does not work')
    args = parser.parse_args()
    stream = NeRFCaptureStream(args.iface, args.peer)
    stream.connect()
    last = None
    while True:
        f = stream._latest()
        if f is not None and f is not last:
            last = f
            depth = stream.get_depth_frame()
            valid = depth[np.isfinite(depth) & (depth > 0)]
            print('frame {}: rgb {}x{}, depth {}x{} [{:.2f}, {:.2f}] m, f=({:.1f}, {:.1f}) c=({:.1f}, {:.1f})'
                  .format(f.id, f.width, f.height, f.depth_width, f.depth_height,
                          valid.min() if valid.size else 0, valid.max() if valid.size else 0,
                          f.fl_x, f.fl_y, f.cx, f.cy))
        time.sleep(0.1)
