"""Webcams read straight from the device, as the camera's own JPEG frames.

The C920 compresses to JPEG itself (its MJPG mode). Taking those frames as
they are costs about 1% of a core per camera; a v4l2_camera node instead
converts every frame to RGB and encodes it to JPEG again, about 40% per camera
at 30 fps on the Jetson. So the web panel reads the cameras itself and the
v4l2_camera nodes are not started (bringup camera_mode:=direct, the default).

Webcams usually leave the Huffman tables out of their MJPG frames (they always
use the standard ones), and not every browser decodes such a frame: the
standard tables are put back in, a copy of a few hundred bytes. A camera that
cannot do MJPG at all has its frames encoded here, at most FPS_MAX a second.

A camera that is unplugged or busy is retried every couple of seconds.
"""

import os
import threading
import time

try:
    import cv2
except ImportError:  # the panel still runs, without direct cameras
    cv2 = None

SOI = b'\xff\xd8'   # start of a JPEG
DHT = b'\xff\xc4'   # Huffman table segment
SOS = b'\xff\xda'   # start of scan: the tables come before it
FPS_MAX = 15.0      # frames kept per second; the page streams at most 10
RETRY_S = 2.0


def _dht(table_class_id, bits, values):
    body = bytes([table_class_id]) + bytes(bits) + bytes(values)
    return DHT + (len(body) + 2).to_bytes(2, 'big') + body


# The standard tables (JPEG spec, Annex K.3), which MJPG cameras use without
# sending them.
_AC_LUMA = bytes.fromhex(
    '01020300041105122131410613516107227114328191a1082342b1c11552d1f0'
    '2433627282090a161718191a25262728292a3435363738393a43444546474849'
    '4a535455565758595a636465666768696a737475767778797a83848586878889'
    '8a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5'
    'c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8'
    'f9fa')
_AC_CHROMA = bytes.fromhex(
    '000102031104052131061241510761711322328108144291a1b1c109233352f0'
    '156272d10a162434e125f11718191a262728292a35363738393a434445464748'
    '494a535455565758595a636465666768696a737475767778797a828384858687'
    '88898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3'
    'c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8'
    'f9fa')
STANDARD_DHT = (
    _dht(0x00, [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], range(12))
    + _dht(0x10, [0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d], _AC_LUMA)
    + _dht(0x01, [0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], range(12))
    + _dht(0x11, [0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77], _AC_CHROMA))


def has_huffman_tables(jpeg):
    sos = jpeg.find(SOS)
    return jpeg.find(DHT, 0, sos if sos > 0 else len(jpeg)) != -1


def with_huffman_tables(jpeg):
    """The frame with the standard tables put in before its scan (None if it has no scan)."""
    sos = jpeg.find(SOS)
    return jpeg[:sos] + STANDARD_DHT + jpeg[sos:] if sos > 0 else None


class DirectCamera:
    def __init__(self, device, on_frame, width=640, height=480, quality=70, log=None):
        """on_frame(jpeg bytes) is called from the camera's thread, at most FPS_MAX times a second."""
        self.device, self.on_frame = device, on_frame
        self.width, self.height, self.quality = width, height, quality
        self.log = log or (lambda msg: None)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f'camera {device}', daemon=True)

    def start(self):
        if cv2 is None:
            self.log('OpenCV (python3-opencv) is missing: no direct cameras')
            return
        self.thread.start()

    def stop(self, timeout=1.0):
        """Stop and wait for the thread: it must not call on_frame after this."""
        self.stopping.set()
        if self.thread.is_alive():
            self.thread.join(timeout)

    def _open(self, raw_jpeg):
        if not os.path.exists(self.device):  # unplugged; OpenCV would warn every retry
            return None
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always the newest frame
        # 0: hand over the camera's JPEG bytes untouched; 1: decoded pixels
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0 if raw_jpeg else 1)
        return cap

    def _encode(self, image):
        ok, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return buf.tobytes() if ok else None

    def _run(self):
        raw_jpeg = True   # until the camera shows it cannot do MJPG
        warned = False
        while not self.stopping.is_set():
            cap = self._open(raw_jpeg)
            if cap is None:
                if not warned:
                    self.log(f'{self.device}: cannot open (unplugged, or used by another program?); retrying')
                    warned = True
                self.stopping.wait(RETRY_S)
                continue
            if warned:
                self.log(f'{self.device}: open')
                warned = False
            last = 0.0
            while not self.stopping.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.log(f'{self.device}: no frame, reopening')
                    break
                now = time.monotonic()
                if now - last < 1.0 / FPS_MAX:
                    continue  # read (the camera runs at its own rate), but keep fewer
                last = now
                if raw_jpeg:
                    jpeg = frame.tobytes()
                    if jpeg[:2] != SOI:  # not MJPG after all: encode here from now on
                        self.log(f'{self.device}: no MJPG, encoding the frames here')
                        raw_jpeg = False
                        break
                    if not has_huffman_tables(jpeg):
                        jpeg = with_huffman_tables(jpeg)
                else:
                    jpeg = self._encode(frame)
                if jpeg and not self.stopping.is_set():
                    self.on_frame(jpeg)
            cap.release()
            self.stopping.wait(0.5)
