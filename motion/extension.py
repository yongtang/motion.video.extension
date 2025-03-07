import asyncio

import omni.ext


class MotionExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[MotionExtension] Extension Startup")
        self._running = True
        self._task = asyncio.ensure_future(self.hello_world_loop())

    def on_shutdown(self):
        print("[MotionExtension] Extension Shut Down")
        self._running = False
        if self._task:
            self._task.cancel()

    async def hello_world_loop(self):
        while self._running:
            print("[MotionExtension] Hello, World!")
            await asyncio.sleep(1)
