# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Unlicense OR CC0-1.0
import argparse
import os
import socket
import sys
import time

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
parser.add_argument('--save-dir', default=None,
                    help='Save every received JPEG into this directory, plus a '
                         'big-endian RGB565 .bin scaled the same way the '
                         'firmware decodes it. Feed the .bin files to '
                         'test/harness for offline line-follow regression.')
parser.add_argument('--save-width', type=int, default=240,
                    help='Width of the .bin dumps written by --save-dir')
parser.add_argument('--save-height', type=int, default=160,
                    help='Height of the .bin dumps written by --save-dir')
args = parser.parse_args()

if args.frames < 0:
    parser.error('--frames must be zero or greater')

def dump_rgb565(image, path, out_w, out_h):
    """Write the frame as big-endian RGB565, matching the firmware buffer."""
    resized = cv2.resize(image, (out_w, out_h), interpolation=cv2.INTER_AREA)
    blue = resized[:, :, 0].astype(np.uint16)
    green = resized[:, :, 1].astype(np.uint16)
    red = resized[:, :, 2].astype(np.uint16)
    packed = ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)
    big_endian = packed.astype('>u2')
    with open(path, 'wb') as handle:
        handle.write(big_endian.tobytes())


if args.save_dir:
    os.makedirs(args.save_dir, exist_ok=True)

frame_count = 0
stream = bytearray()
saved = False
stop = False
ball_markers = {}
ball_phase = 'IDLE'
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
                    elif line.startswith(b'DETECT '):
                        fields = line.decode('ascii', errors='replace').split()
                        if len(fields) >= 2 and fields[1] == 'CLEAR':
                            ball_markers.clear()
                            ball_phase = 'IDLE'
                        elif len(fields) >= 7:
                            try:
                                colour = fields[1].lower()
                                # DETECT coordinates are in the control frame
                                # used by the ESP32 detector. New firmware
                                # appends that frame's width and height; keep
                                # the known 120x80 mode as a compatibility
                                # fallback for older firmware.
                                frame_w = int(fields[7]) if len(fields) >= 9 else 120
                                frame_h = int(fields[8]) if len(fields) >= 9 else 80
                                if frame_w <= 0 or frame_h <= 0:
                                    frame_w, frame_h = 120, 80
                                ball_markers[colour] = {
                                    'x': int(fields[2]), 'y': int(fields[3]),
                                    'w': int(fields[4]), 'h': int(fields[5]),
                                    'phase': fields[6], 'frame_w': frame_w,
                                    'frame_h': frame_h, 'time': time.monotonic(),
                                }
                                ball_phase = fields[6]
                            except ValueError:
                                pass

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

                if args.save_dir:
                    stem = os.path.join(args.save_dir, f'frame_{frame_count:05d}')
                    with open(stem + '.jpg', 'wb') as handle:
                        handle.write(jpg)
                    dump_rgb565(image, stem + '.bin', args.save_width,
                                args.save_height)

                frame_count += 1
                if not args.headless:
                    # DETECT coordinates are from the ESP32 control image;
                    # scale them to the full-resolution JPEG shown here.
                    image_h, image_w = image.shape[:2]
                    for colour, marker in list(ball_markers.items()):
                        if time.monotonic() - marker['time'] > 2.0:
                            del ball_markers[colour]
                            continue
                        sx = image_w / marker['frame_w']
                        sy = image_h / marker['frame_h']
                        cx = int(marker['x'] * sx)
                        cy = int(marker['y'] * sy)
                        half_w = max(4, int(marker['w'] * sx / 2.0))
                        half_h = max(4, int(marker['h'] * sy / 2.0))
                        color = (0, 0, 255) if colour == 'red' else (255, 180, 0)
                        cv2.rectangle(image, (cx - half_w, cy - half_h),
                                      (cx + half_w, cy + half_h), color, 2)
                        cv2.drawMarker(image, (cx, cy), color,
                                       cv2.MARKER_CROSS, 14, 2)
                        cv2.putText(image, f'{colour.upper()} {marker["phase"]}',
                                    (max(0, cx - half_w), max(18, cy - half_h - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
                                    cv2.LINE_AA)
                    cv2.putText(image, f'ESP32: {ball_phase}', (8, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
                                cv2.LINE_AA)
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
