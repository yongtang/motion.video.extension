import asyncio
from .websocket import start_websocket_server, stop_websocket_server, received_data
import omni.ext


class MotionExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[MotionExtension] Extension Startup")
        self._running = True
        self._task = asyncio.ensure_future(self.hello_world_loop())

        start_websocket_server()

        # Subscribe to Isaac Sim's update loop
        self._update_task = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self.update)
        )

    def on_shutdown(self):
        print("[MotionExtension] Extension Shut Down")
        self._running = False
        if self._task:
            self._task.cancel()

        stop_websocket_server()

    def update(self, e):
        """This function runs every frame in Isaac Sim"""
        if received_data["message"]:
            print(f"[MyExtension] WebSocket data: {received_data['message']}")

    async def hello_world_loop(self):
        while self._running:
            print("[MotionExtension] Hello, World!")
            await asyncio.sleep(1)
