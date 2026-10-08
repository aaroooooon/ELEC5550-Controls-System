#!/usr/bin/env python3
"""
Viewer for the XIAO ESP32S3 Sense weed-detection stream.

The board writes framed packets to its USB-Serial-JTAG port (see src/main.cpp),
a detections packet followed by the JPEG it belongs to:

    magic "DETS" | uint32 little-endian length | detections payload
    magic "JPGF" | uint32 little-endian length | JPEG payload

This reads that stream, resynchronising on the magics so it can be started and
stopped at any time, and shows the frames in a window with the model's boxes
drawn over them. The dashed square is the part of the frame the model sees.
Older firmware that sends only JPGF packets still displays, without boxes.

    python tools/view.py                  # auto-detect the board
    python tools/view.py --port COM3
    python tools/view.py --save-dir shots # 's' writes a JPEG there

Keys: q or Esc to quit, s to save the current frame (as sent, no drawing),
b to toggle the boxes, f to toggle the stats line.
"""

import argparse
import collections
import struct
import sys
import time
from pathlib import Path

try:
    import cv2
    import numpy as np
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    sys.exit(
        f"missing dependency: {exc.name}\n"
        f"install with:  python -m pip install -r tools/requirements.txt"
    )

JPEG_MAGIC = b"JPGF"
DETS_MAGIC = b"DETS"
MAGICS = (JPEG_MAGIC, DETS_MAGIC)
MAGIC_BYTES = 4
HDR_BYTES = 8

# A VGA frame at quality 12 is ~20-30 KB. Anything wildly outside this is a bad
# length field from a corrupt header, so the parser resyncs instead of stalling
# while it waits for bytes that will never arrive.
MIN_FRAME_BYTES = 1024
MAX_FRAME_BYTES = 512 * 1024

# A detections packet is a fixed header, the label names, and 10 bytes a box.
DETS_FIXED_BYTES = 17
MIN_DETS_BYTES = DETS_FIXED_BYTES + 2
MAX_DETS_BYTES = 16 * 1024
DETS_BOX_BYTES = 10

# BGR colour for each known label, by name, so a label keeps its colour when a
# retrained model adds labels and shifts the indices. Names must match the
# Edge Impulse labels exactly; the board sends them in every DETS packet.
LABEL_COLOURS_BY_NAME = {
    "Brass Button": (0, 200, 255),    # amber
    "Burr medic":   (255, 120, 0),    # blue
    "Clover":       (60, 220, 60),    # green
    "Dandelion":    (140, 140, 255),  # pink (added with model deploy v5)
    "Flat Weed":    (230, 60, 230),   # magenta
    "Tall Weed":    (0, 90, 255),     # orange
}

# Fallback for labels not listed above: one BGR colour per label index, cycled
# if the model has more labels.
LABEL_COLOURS = [
    (0, 200, 255),    # amber
    (255, 120, 0),    # blue
    (60, 220, 60),    # green
    (230, 60, 230),   # magenta
    (0, 90, 255),     # orange
    (255, 230, 0),    # cyan
    (140, 140, 255),  # pink
]

# Espressif's USB vendor ID: the XIAO ESP32S3 enumerates natively, not through
# a CH340/CP210x bridge, so the VID belongs to the chip itself.
ESPRESSIF_VID = 0x303A


def find_port() -> str:
    """Pick the board's port, preferring an Espressif VID match."""
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("no serial ports found - is the board plugged in?")

    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device

    if len(ports) == 1:
        return ports[0].device

    listing = "\n".join(f"  {p.device}  {p.description}" for p in ports)
    sys.exit(f"could not identify the board. Pass --port explicitly:\n{listing}")


def try_reopen(port_arg):
    """Reopen the board's port after it dropped off the bus, or return None.

    The board re-enumerates whenever it resets (RESET button, a flash, or a
    crash), and may come back under a different COM number if it was
    auto-detected."""
    port = port_arg
    if port is None:
        port = next((p.device for p in list_ports.comports()
                     if p.vid == ESPRESSIF_VID), None)
        if port is None:
            return None
    try:
        return serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException:
        return None


class Detections:
    """One frame's worth of model output, as decoded from a DETS packet."""

    def __init__(self, frame_w, frame_h, roi, model_ms, threshold, gpio_high,
                 labels, boxes):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.roi = roi              # (x, y, w, h): the part of the frame the model sees
        self.model_ms = model_ms
        self.threshold = threshold  # detections in one frame that drive GPIO1 high
        self.gpio_high = gpio_high
        self.labels = labels
        self.boxes = boxes          # [(label, confidence 0-255, x, y, w, h)]


def dets_header_plausible(hdr: bytes) -> bool:
    """Early check on a DETS header, so a false match is dropped at once rather
    than after buffering up to MAX_DETS_BYTES of whatever follows it."""
    frame_w, frame_h = struct.unpack_from("<HH", hdr, 0)
    return 0 < frame_w <= 4096 and 0 < frame_h <= 4096 and hdr[16] <= 64


def parse_detections(payload: bytes):
    """Decode a DETS payload, or return None if it is not a well-formed one."""
    try:
        (frame_w, frame_h, rx, ry, rw, rh, model_ms,
         threshold, flags, label_count) = struct.unpack_from("<7HBBB", payload, 0)
        pos = DETS_FIXED_BYTES

        labels = []
        for _ in range(label_count):
            n = payload[pos]
            labels.append(payload[pos + 1:pos + 1 + n].decode("utf-8", "replace"))
            pos += 1 + n

        (count,) = struct.unpack_from("<H", payload, pos)
        pos += 2
        if pos + count * DETS_BOX_BYTES != len(payload):
            return None

        boxes = [struct.unpack_from("<BBHHHH", payload, pos + i * DETS_BOX_BYTES)
                 for i in range(count)]
    except (IndexError, struct.error):
        return None

    return Detections(frame_w, frame_h, (rx, ry, rw, rh), model_ms, threshold,
                      bool(flags & 0x01), labels, boxes)


class FrameParser:
    """Incremental parser: feed it bytes, get ("jpeg", bytes) and
    ("dets", Detections) items back."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.resyncs = 0
        self.skipped_bytes = 0

    def _find_magic(self) -> int:
        hits = [i for i in (self.buf.find(m) for m in MAGICS) if i >= 0]
        return min(hits) if hits else -1

    def _reject_magic(self) -> None:
        """This "magic" was really payload bytes. Step past it and rescan."""
        self.skipped_bytes += MAGIC_BYTES
        self.resyncs += 1
        del self.buf[:MAGIC_BYTES]

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

        while True:
            start = self._find_magic()
            if start < 0:
                # No header in view. Keep the last few bytes in case a magic
                # straddles this chunk and the next.
                if len(self.buf) > MAGIC_BYTES:
                    drop = len(self.buf) - MAGIC_BYTES + 1
                    self.skipped_bytes += drop
                    del self.buf[:drop]
                return

            if start > 0:
                # Console output or a partial packet ahead of the header.
                self.skipped_bytes += start
                self.resyncs += 1
                del self.buf[:start]

            if len(self.buf) < HDR_BYTES:
                return

            is_jpeg = self.buf[:MAGIC_BYTES] == JPEG_MAGIC
            length = int.from_bytes(self.buf[MAGIC_BYTES:HDR_BYTES], "little")

            # A length outside the plausible range means a corrupt header;
            # resync rather than stall waiting for bytes that will never come.
            if is_jpeg:
                length_ok = MIN_FRAME_BYTES <= length <= MAX_FRAME_BYTES
            else:
                length_ok = MIN_DETS_BYTES <= length <= MAX_DETS_BYTES
            if not length_ok:
                self._reject_magic()
                continue

            # Check the start of the payload as soon as it lands rather than
            # buffering the whole length first: a header that is not followed
            # by a plausible payload was never a header.
            if is_jpeg and len(self.buf) >= HDR_BYTES + 2:
                if self.buf[HDR_BYTES:HDR_BYTES + 2] != b"\xff\xd8":
                    self._reject_magic()
                    continue
            if not is_jpeg and len(self.buf) >= HDR_BYTES + DETS_FIXED_BYTES:
                if not dets_header_plausible(bytes(self.buf[HDR_BYTES:HDR_BYTES + DETS_FIXED_BYTES])):
                    self._reject_magic()
                    continue

            end = HDR_BYTES + length
            if len(self.buf) < end:
                return  # packet still arriving

            payload = bytes(self.buf[HDR_BYTES:end])

            if is_jpeg and payload[-2:] == b"\xff\xd9":
                del self.buf[:end]
                yield "jpeg", payload
                continue

            if not is_jpeg:
                # The board always sends a frame's JPGF straight after its DETS.
                # Checking for it catches a cut-short DETS whose declared length
                # ran into the next packet, which can otherwise still parse.
                if len(self.buf) < end + MAGIC_BYTES:
                    return
                if self.buf[end:end + MAGIC_BYTES] != JPEG_MAGIC:
                    self._reject_magic()
                    continue

                dets = parse_detections(payload)
                if dets is not None:
                    del self.buf[:end]
                    yield "dets", dets
                    continue

            # The board's write was cut short, so this packet ran into whatever
            # followed it. Consuming `length` bytes here would eat the next
            # packet's header along with it, so give up only the magic and
            # rescan - the real header is somewhere in the bytes we just spanned.
            self._reject_magic()


def put_tag(img, text, x, y, colour, scale=0.45) -> None:
    """Text on a filled tag whose bottom-left corner is at (x, y)."""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    y = max(y, th + base + 2)
    cv2.rectangle(img, (x, y - th - base - 2), (x + tw + 4, y), colour, cv2.FILLED)
    cv2.putText(img, text, (x + 2, y - base), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), 1, cv2.LINE_AA)


def draw_dashed_rect(img, x, y, w, h, colour, dash=8) -> None:
    for x0 in range(x, x + w, dash * 2):
        x1 = min(x0 + dash, x + w - 1)
        cv2.line(img, (x0, y), (x1, y), colour, 1)
        cv2.line(img, (x0, y + h - 1), (x1, y + h - 1), colour, 1)
    for y0 in range(y, y + h, dash * 2):
        y1 = min(y0 + dash, y + h - 1)
        cv2.line(img, (x, y0), (x, y1), colour, 1)
        cv2.line(img, (x + w - 1, y0), (x + w - 1, y1), colour, 1)


def draw_detections(img, dets: Detections) -> None:
    # Coordinates are in the board's frame pixels; scale in case the image
    # decoded at a different size.
    sx = img.shape[1] / dets.frame_w
    sy = img.shape[0] / dets.frame_h

    rx, ry, rw, rh = dets.roi
    draw_dashed_rect(img, int(rx * sx), int(ry * sy), int(rw * sx), int(rh * sy),
                     (200, 200, 200))

    for label, conf, x, y, w, h in dets.boxes:
        if label < len(dets.labels):
            name = dets.labels[label]
            colour = LABEL_COLOURS_BY_NAME.get(
                name, LABEL_COLOURS[label % len(LABEL_COLOURS)])
        else:
            name = f"class {label}"
            colour = (255, 255, 255)
        x0, y0 = int(x * sx), int(y * sy)
        x1, y1 = int((x + w) * sx), int((y + h) * sy)
        cv2.rectangle(img, (x0, y0), (x1, y1), colour, 2)
        put_tag(img, f"{name} {conf * 100 // 255}%", x0, y0 - 1, colour)

    # The board holds GPIO1 high for a while after the last frame that met the
    # threshold, so a frame below it can still show the pin high.
    if not dets.gpio_high:
        gpio, colour = "low", (0, 200, 0)
    elif len(dets.boxes) >= dets.threshold:
        gpio, colour = "HIGH", (0, 0, 255)
    else:
        gpio, colour = "HIGH (hold from earlier frame)", (0, 165, 255)
    banner = f"weeds: {len(dets.boxes)}  (trigger at {dets.threshold})   GPIO1: {gpio}"
    (tw, th), base = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    y = img.shape[0] - 10
    cv2.rectangle(img, (4, y - th - base - 4), (tw + 16, y + 4), (0, 0, 0), cv2.FILLED)
    cv2.putText(img, banner, (10, y - base + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                colour, 2, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--save-dir", default=".", help="where 's' writes frames")
    ap.add_argument("--no-overlay", action="store_true", help="start with stats hidden")
    ap.add_argument("--no-boxes", action="store_true", help="start with boxes hidden")
    args = ap.parse_args()

    port = args.port or find_port()
    save_dir = Path(args.save_dir)

    # Baud rate is ignored by a native USB CDC device, but pyserial wants one.
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as exc:
        return f"could not open {port}: {exc}\nIs a serial monitor still holding it open?"

    print(f"reading {port} - press q in the window to quit")

    parser = FrameParser()
    times: collections.deque = collections.deque(maxlen=30)
    overlay = not args.no_overlay
    show_boxes = not args.no_boxes
    pending_dets = None  # detections for the JPEG that follows them
    frames = 0
    decode_errors = 0
    saved = 0
    window = "XIAO ESP32S3 Sense - weed detection"
    last_frame_at = time.monotonic()
    warned_idle = False
    next_reopen = 0.0

    try:
        while True:
            chunk = b""
            if ser is None:
                if time.monotonic() >= next_reopen:
                    ser = try_reopen(args.port)
                    next_reopen = time.monotonic() + 0.5
                    if ser is not None:
                        print(f"reconnected on {ser.port}")
                        parser = FrameParser()
                        pending_dets = None
                        last_frame_at = time.monotonic()
                        warned_idle = False
            else:
                try:
                    # read() with a timeout returns as soon as data is there;
                    # in_waiting lets a backlog come across in one go instead
                    # of 4 KB at a time.
                    chunk = ser.read(max(4096, ser.in_waiting))
                except serial.SerialException:
                    print(f"lost {ser.port} - the board reset or rebooted. "
                          "Waiting for it to come back (q to quit)...")
                    ser.close()
                    ser = None
                    continue

            if chunk:
                for kind, item in parser.feed(chunk):
                    if kind == "dets":
                        pending_dets = item
                        continue

                    payload = item
                    dets, pending_dets = pending_dets, None
                    img = cv2.imdecode(np.frombuffer(payload, np.uint8),
                                       cv2.IMREAD_COLOR)
                    if img is None:
                        decode_errors += 1
                        continue

                    frames += 1
                    now = time.monotonic()
                    times.append(now)
                    last_frame_at = now
                    warned_idle = False

                    if overlay:
                        fps = 0.0
                        if len(times) > 1:
                            span = times[-1] - times[0]
                            if span > 0:
                                fps = (len(times) - 1) / span
                        label = (f"{fps:4.1f} fps  {img.shape[1]}x{img.shape[0]}  "
                                 f"{len(payload) / 1024:.0f} KB  "
                                 f"{fps * len(payload) / 1024:.0f} KB/s")
                        if dets is not None:
                            label += f"  model {dets.model_ms} ms"
                        cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
                        cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 128), 1, cv2.LINE_AA)

                    if show_boxes and dets is not None:
                        draw_detections(img, dets)

                    cv2.imshow(window, img)
                    last_payload = payload

            elif ser is not None and time.monotonic() - last_frame_at > 3.0 and not warned_idle:
                print("no frames for 3s - is the board running the weed-detection build? "
                      "Tap RESET to restart it.")
                warned_idle = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("f"):
                overlay = not overlay
            if key == ord("b"):
                show_boxes = not show_boxes
            if key == ord("s") and frames:
                save_dir.mkdir(parents=True, exist_ok=True)
                path = save_dir / f"frame_{time.strftime('%Y%m%d_%H%M%S')}_{saved:03d}.jpg"
                path.write_bytes(last_payload)  # the original bytes, no overlay
                saved += 1
                print(f"saved {path}")

            # Closing the window with the X button should quit too.
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1 and frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()

    print(f"{frames} frames, {decode_errors} decode errors, "
          f"{parser.resyncs} resyncs, {parser.skipped_bytes} bytes skipped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
