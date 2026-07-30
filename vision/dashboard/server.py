"""
FastAPI + WebSocket dashboard: streams the hand-tracker's video frames and telemetry
(finger curls, FPS, serial/gazebo link status) to a browser tab.

Runs in its own background thread (see run_dashboard_server / hand_tracker.py's
--dashboard flag) alongside the existing synchronous capture loop and cv2 window -
this only adds a second output sink, it doesn't replace the local window.
"""

import asyncio
import threading
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

STATIC_DIR = Path(__file__).parent / "static"


class DashboardHub:
    """Bridges the synchronous capture-loop thread to the async websocket clients.

    publish() is called once per frame from the (sync) capture thread; it hands each
    connected client's payload off via loop.call_soon_threadsafe, since asyncio.Queue
    isn't safe to touch directly from a non-event-loop thread.
    """

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
        # Drop the previous unread frame rather than let the queue back up -
        # the dashboard only ever wants the latest frame, not a backlog.
        if queue.full():
            queue.get_nowait()
        queue.put_nowait(payload)


def create_app(hub: DashboardHub) -> FastAPI:
    app = FastAPI()

    @app.on_event("startup")
    async def _bind_loop():
        hub.bind_loop(asyncio.get_running_loop())

    @app.get("/")
    async def index():
        return FileResponse(STATIC_DIR / "index.html")

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

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

    return app


def run_dashboard_server(hub: DashboardHub, host="0.0.0.0", port=8765):
    app = create_app(hub)
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    server.run()
