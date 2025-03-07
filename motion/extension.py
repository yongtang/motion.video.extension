import asyncio
import omni.ext
from . import websocket


class MotionExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[MotionExtension] Extension startup")
        self.server = websocket.Server()
        asyncio.run(self.server.startup())

    def on_shutdown(self):
        print("[MotionExtension] Extension shutdown")
        asyncio.run(self.server.shutdown())
        stop_websocket_server()
