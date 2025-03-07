import omni.ext
import asyncio, uvicorn
from . import websocket


class MotionExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.server = asyncio.create_task(
            uvicorn.Server(
                uvicorn.Config(
                    websocket.app,
                    host="0.0.0.0",
                    port=5000,
                    log_level="info",
                    loop="asyncio",
                )
            ).serve()
        )
        print("[MotionExtension] Extension startup")

    def on_shutdown(self):
        if getattr(self, "server", None):
            self.server.cancel()
            try:
                await self.server
            except asyncio.CancelledError:
                pass
            self.server = None
            print("[MotionExtension] Server shutdown")
        print("[MotionExtension] Extension shutdown")
