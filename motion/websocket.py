from fastapi import FastAPI, WebSocket
import uvicorn
import asyncio

app = FastAPI()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        print(f"Received from WebSocket: {data}")
        await websocket.send_text(f"Message received: {data}")


@app.get("/health")
async def health():
    return {"status": "ok"}


class Server:
    def __init__(self, host="0.0.0.0", port=5000):
        self.host = host
        self.port = port
        self.task = None

    async def startup(self):
        if self.task is None:
            config = uvicorn.Config(
                app, host=self.host, port=self.port, log_level="info", loop="asyncio"
            )
            server = uvicorn.Server(config)
            self.task = asyncio.create_task(server.serve())
            print("[WebSocket] Server startup")

    async def shutdown(self):
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
            self.task = None
            print("[WebSocket] Server shutdown")


# uvicorn motion.websocket:app --host 0.0.0.0 --port 8000 --reload
