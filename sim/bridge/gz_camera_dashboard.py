"""
Runs INSIDE the robotics-gazebo container (needs gz.transport13/gz.msgs10, same as
gz_hand_bridge.py). Subscribes to all six camera sensors' image topics in hand_world.sdf
(overview/left_hand/right_hand, each with a front and a back variant - see the mirrored
back_* camera models added alongside issue #7's front/back follow-up) and re-serves each
as its own WebSocket JPEG stream on the container's host network (--network=host, see
sim/docker/run.sh). One gz-transport subscription per camera, one CameraHub per camera -
the browser picks which feed to watch via the URL path (/ws/<camera>/<side>), not a
filter on a single shared stream.

Unlike gz_hand_bridge.py this isn't piped from another process's stdin - it has no
input, just topic subscriptions - so it's launched standalone and detached, independent
of vision/hand_tracker.py's lifecycle:

    docker exec -d robotics_gazebo_sim python3 /sim/bridge/gz_camera_dashboard.py

(or sim/bridge/start_camera_dashboard.sh). The vision dashboard's browser page opens a
second WebSocket directly to this server's port - the two dashboards are independent
processes/servers, not one connecting through the other (see CLAUDE.md's "sim runs its
own server" decision).
"""

import asyncio
import io
import threading

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from PIL import Image as PILImage

# (camera, side) -> gz-transport image topic. "camera" and "side" are exactly the path
# segments used in /ws/<camera>/<side>.
CAMERAS = {
    ("overview", "front"): "overview_camera/image",
    ("left", "front"): "left_hand_camera/image",
    ("right", "front"): "right_hand_camera/image",
    ("overview", "back"): "back_overview_camera/image",
    ("left", "back"): "back_left_hand_camera/image",
    ("right", "back"): "back_right_hand_camera/image",
}
PORT = 8766


class CameraHub:
    """Same broadcast shape as vision/dashboard/server.py's DashboardHub - can't import
    that module directly, this runs in a separate container/Python environment."""

    def __init__(self):
        self._loop = None
        self._queues = set()
        self._lock = threading.Lock()

    def bind_loop(self, loop):
        self._loop = loop

    def connect(self):
        queue = asyncio.Queue(maxsize=1)
        with self._lock:
            self._queues.add(queue)
        return queue

    def disconnect(self, queue):
        with self._lock:
            self._queues.discard(queue)

    def has_subscribers(self):
        with self._lock:
            return bool(self._queues)

    def publish(self, payload):
        if self._loop is None:
            return
        with self._lock:
            queues = list(self._queues)
        for queue in queues:
            self._loop.call_soon_threadsafe(self._offer, queue, payload)

    @staticmethod
    def _offer(queue, payload):
        if queue.full():
            queue.get_nowait()
        queue.put_nowait(payload)


hubs = {key: CameraHub() for key in CAMERAS}
app = FastAPI()


def make_on_image(key):
    hub = hubs[key]

    def on_image(msg):
        # Skip the decode/encode work entirely when nobody's watching this camera - all
        # six sensors publish at 30Hz regardless of the browser's current selection (see
        # CAMERAS above), and encoding all six every frame was starving whichever one
        # was actually being viewed (GIL contention with gz-sim's own render/physics
        # load in this same container). Only the encode is gated, not the gz-transport
        # subscription itself - cheap to leave that open.
        if not hub.has_subscribers():
            return
        # <format>R8G8B8</format> in the sensor's SDF -> raw interleaved RGB bytes.
        img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        # Raw bytes over a binary WebSocket frame, not base64-in-JSON - skips both the
        # base64 CPU pass and its ~33% size overhead.
        hub.publish(buf.getvalue())

    return on_image


@app.on_event("startup")
async def _bind_loops():
    loop = asyncio.get_running_loop()
    for hub in hubs.values():
        hub.bind_loop(loop)


@app.get("/")
async def health():
    return {"status": "ok", "cameras": {f"{c}/{s}": t for (c, s), t in CAMERAS.items()}}


@app.websocket("/ws/{camera}/{side}")
async def ws_endpoint(websocket: WebSocket, camera: str, side: str):
    key = (camera, side)
    if key not in hubs:
        await websocket.close(code=1008)
        return
    await websocket.accept()
    queue = hubs[key].connect()
    try:
        while True:
            payload = await queue.get()
            await websocket.send_bytes(payload)
    except WebSocketDisconnect:
        pass
    finally:
        hubs[key].disconnect(queue)


# Bare /ws keeps the pre-switcher default (front overview) working for anything not yet
# updated to the per-camera/per-side path.
@app.websocket("/ws")
async def ws_default(websocket: WebSocket):
    await ws_endpoint(websocket, "overview", "front")


def main():
    node = Node()
    for key, topic in CAMERAS.items():
        node.subscribe(Image, topic, make_on_image(key))
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")


if __name__ == "__main__":
    main()
