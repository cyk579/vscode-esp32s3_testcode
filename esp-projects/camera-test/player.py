# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Unlicense OR CC0-1.0
import argparse
import socket
import sys

try:
    import cv2
    import numpy as np
except ModuleNotFoundError as error:
    print(f'Current Python: {sys.executable}')
    print(f'Missing module: {error.name}')
    print('Run this player with: py -3.13 player.py')
    print('Or install dependencies into this exact Python with:')
    print(f'"{sys.executable}" -m pip install opencv-python numpy')
    raise SystemExit(1) from error

parser = argparse.ArgumentParser(description='Display MJPEG frames from ESP32 camera test')
parser.add_argument('--host', default='192.168.4.1')
parser.add_argument('--port', type=int, default=2222)
parser.add_argument('--save', default='camera_capture.jpg',
                    help='Save the first valid frame to this file')
parser.add_argument('--headless', action='store_true',
                    help='Do not open a window; useful for an automatic capture check')
parser.add_argument('--frames', type=int, default=0,
                    help='Stop after this many valid frames (0 means keep running)')
args = parser.parse_args()

if args.frames < 0:
    parser.error('--frames must be zero or greater')

frame_count = 0
stream = bytearray()
saved = False
stop = False
max_frames = args.frames or (1 if args.headless else 0)
max_stream_bytes = 2 * 1024 * 1024

print(f'Connecting to {args.host}:{args.port}...')

try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((args.host, args.port))
        print('Receiving MJPEG data. Press Esc in the preview window to exit.')

        while not stop:
            data = sock.recv(4096)
            if not data:
                break
            stream += data

            while True:
                # Firmware status lines are placed before JPEG frames. They
                # make USB wiring and UVC negotiation failures visible even
                # when no image is received yet.
                while True:
                    newline = stream.find(b'\n')
                    soi = stream.find(b'\xff\xd8')
                    if newline == -1 or (soi != -1 and newline > soi):
                        break
                    line = bytes(stream[:newline]).strip()
                    del stream[:newline + 1]
                    if line.startswith(b'STATUS:'):
                        print(f'ESP32 status: {line.decode("ascii", errors="replace")}')

                soi = stream.find(b'\xff\xd8')
                if soi == -1:
                    # Keep one byte in case a JPEG SOI marker crosses recv()
                    # boundaries, but do not let corrupt data grow forever.
                    if len(stream) > max_stream_bytes:
                        print('Discarding oversized non-JPEG input')
                        del stream[:-1]
                    break
                if soi > 0:
                    del stream[:soi]

                eoi = stream.find(b'\xff\xd9', 2)
                if eoi == -1:
                    if len(stream) > max_stream_bytes:
                        print('Discarding oversized incomplete JPEG frame')
                        del stream[:2]
                    break

                jpg = bytes(stream[:eoi + 2])
                del stream[:eoi + 2]
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    print('Invalid JPEG frame received')
                    continue

                if frame_count == 0 and args.save:
                    saved = cv2.imwrite(args.save, image)
                    if saved:
                        print(f'First frame saved to {args.save}')
                    else:
                        print(f'Failed to save first frame to {args.save}')

                frame_count += 1
                if not args.headless:
                    cv2.imshow('ESP32 USB camera', image)
                    if cv2.waitKey(1) == 27:
                        stop = True
                        break
                if max_frames and frame_count >= max_frames:
                    stop = True
                    break
finally:
    cv2.destroyAllWindows()

print(f'Frames received: {frame_count}; first frame saved: {saved}')
