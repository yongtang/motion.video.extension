import asyncio
import threading
import time
from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

# Store received WebSocket data
received_data = {"message": None, "last_updated": None}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        print(f"Received from WebSocket: {data}")
        received_data["message"] = data
        received_data["last_updated"] = time.time()
        await websocket.send_text(f"Message received: {data}")


@app.get("/health")
async def health():
    """Simple endpoint to check if API is running"""
    return {"status": "ok"}


@app.get("/status")
async def status():
    """Detailed system status with WebSocket data"""
    uptime = (
        time.time() - received_data["last_updated"]
        if received_data["last_updated"]
        else "N/A"
    )
    return {
        "status": "ok",
        "last_message": received_data["message"],
        "last_updated": uptime,
    }


# Server shutdown handling
server = None  # Global reference to Uvicorn server


async def start_server():
    """Start Uvicorn in an asyncio task"""
    global server
    config = uvicorn.Config(
        app, host="0.0.0.0", port=5000, log_level="info", loop="asyncio"
    )
    server = uvicorn.Server(config)
    await server.serve()


def run_server():
    """Run the server in a separate event loop thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_server())


def start_websocket_server():
    """Start the WebSocket server in a new thread"""
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()


def stop_websocket_server():
    """Stop the server gracefully"""
    global server
    if server:
        asyncio.run(server.shutdown())  # Properly stop Uvicorn
