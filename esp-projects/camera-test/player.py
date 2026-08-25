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
args = parser.parse_args()

frame_count = 0
stream = bytearray()

print(f'Connecting to {args.host}:{args.port}...')

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((args.host, args.port))

    print('Receiving data ')
    while True:
        data = sock.recv(4096)
        if not data:
            break
        stream += data
        print('.', end='', flush=True)

        # Firmware status lines are sent before JPEG frames and do not affect
        # the MJPEG parser. They reveal whether USB enumeration succeeded.
        while True:
            newline = stream.find(b'\n')
            soi = stream.find(b'\xff\xd8')
            if newline == -1 or (soi != -1 and newline > soi):
                break
            line = bytes(stream[:newline]).strip()
            del stream[:newline + 1]
            if line.startswith(b'STATUS:'):
                print(f'\nESP32 status: {line.decode("ascii", errors="replace")}')

        a = stream.find(b'\xff\xd8')
        b = stream.find(b'\xff\xd9', a)

        if a != -1 and b != -1:
            jpg = stream[a:b + 2]
            stream = stream[b + 2:]
            buffer = np.frombuffer(jpg, dtype=np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if image is None:
                print('\nInvalid JPEG frame received')
                continue
            if frame_count == 0 and args.save:
                if cv2.imwrite(args.save, image):
                    print(f'\nFirst frame saved to {args.save}')
                else:
                    print(f'\nFailed to save first frame to {args.save}')
            cv2.imshow('ESP32 USB camera', image)
            if cv2.waitKey(10) == 27:
                exit(0)
            frame_count += 1

print('\nFrames received ', frame_count)
