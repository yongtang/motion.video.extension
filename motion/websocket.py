from fastapi import FastAPI, WebSocket
import uvicorn
import asyncio

app = FastAPI()


@app.on_event("startup")
async def on_startup():
    print("[WebSocket] Server startup")


@app.on_event("shutdown")
async def on_shutdown():
    print("[WebSocket] Server shutdown")


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


# uvicorn motion.websocket:app --host 0.0.0.0 --port 8000 --reload
