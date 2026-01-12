import asyncio
import websockets
import json

async def test_client():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        print("Connected to tracker")
        while True:
            msg = await websocket.recv()
            data = json.loads(msg)
            # print(f"Received: MaxVal={data.get('max_val')} Locked={data.get('locked')}")

asyncio.run(test_client())
