"""
Runs INSIDE the robotics-gazebo container (needs gz.transport13/gz.msgs10, same as
gz_hand_bridge.py). Subscribes to the overview_camera sensor's image topic (added to
hand_world.sdf) and re-serves it as a WebSocket JPEG stream on the container's host
network (--network=host, see sim/docker/run.sh).

Unlike gz_hand_bridge.py this isn't piped from another process's stdin - it has no
input, just a topic subscription - so it's launched standalone and detached, independent
of vision/hand_tracker.py's lifecycle:

    docker exec -d robotics_gazebo_sim python3 /sim/bridge/gz_camera_dashboard.py

(or sim/bridge/start_camera_dashboard.sh). The vision dashboard's browser page opens a
second WebSocket directly to this server's port - the two dashboards are independent
processes/servers, not one connecting through the other (see CLAUDE.md's "sim runs its
own server" decision).
"""

import asyncio
import base64
import io
import threading

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from PIL import Image as PILImage

TOPIC = "overview_camera/image"
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


hub = CameraHub()
app = FastAPI()


def on_image(msg):
    # <format>R8G8B8</format> in the sensor's SDF -> raw interleaved RGB bytes.
    img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    image_b64 = base64.b64encode(buf.getvalue()).decode("ascii")
    hub.publish({"image": f"data:image/jpeg;base64,{image_b64}"})


@app.on_event("startup")
async def _bind_loop():
    hub.bind_loop(asyncio.get_running_loop())


@app.get("/")
async def health():
    return {"status": "ok", "topic": TOPIC}


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    queue = hub.connect()
    try:
        while True:
            payload = await queue.get()
            await websocket.send_json(payload)
    except WebSocketDisconnect:
        pass
    finally:
        hub.disconnect(queue)


def main():
    node = Node()
    node.subscribe(Image, TOPIC, on_image)
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")


if __name__ == "__main__":
    main()
