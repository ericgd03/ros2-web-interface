import asyncio
import websockets
import cv2
import numpy as np
import base64

async def receive_frames():
    uri = "ws://192.168.1.66:8765"
    async with websockets.connect(uri) as websocket:
        print("[✓] Connected to server")

        while True:
            try:
                data = await websocket.recv()
                jpg_bytes = base64.b64decode(data)
                np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                frame = cv2.resize(frame, (640, 320))

                cv2.imshow("Received Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except websockets.exceptions.ConnectionClosed:
                print("[-] Server disconnected")
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_frames())