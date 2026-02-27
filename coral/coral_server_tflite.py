#!/usr/bin/env python3
"""
Coral Server - Simplified version using TensorFlow Lite Runtime directly
This version works with the Coral TPU but uses the lower-level TensorFlow Lite API.

Supports both:
- tflite_runtime (lightweight, tflite-only)
- tensorflow (heavier, but more compatible)

To use the full pycoral library with adapters, install from source:
pip install git+https://github.com/google-coral/pycoral.git
"""

import socket
import struct
import msgpack
import numpy as np
import cv2
from typing import Optional

# Try importing tflite_runtime first, fall back to tensorflow if not available
try:
    import tflite_runtime.interpreter as tflite
    print("[coral_server] Using tflite_runtime", flush=True)
except ImportError:
    try:
        import tensorflow.lite as tflite
        print("[coral_server] Using tensorflow.lite (tflite_runtime not available)", flush=True)
    except ImportError:
        print("[coral_server] ERROR: Neither tflite_runtime nor tensorflow is installed!", flush=True)
        raise

HOST = "0.0.0.0"
PORT = 9900

# Model paths - in the same directory as this script
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_CPU = os.path.join(SCRIPT_DIR, "deeplabv3_mnv2_pascal_quant.tflite")
MODEL_TPU = os.path.join(SCRIPT_DIR, "deeplabv3_mnv2_pascal_quant_edgetpu.tflite")

PERSON_CLASS_ID = 15  # PASCAL VOC "person"
MAX_SIDE_BEFORE_MODEL_RESIZE = 640
CONN_TIMEOUT_S = 10.0

# Set to True only if using Coral USB Accelerator and pycoral is installed
USE_TPU = False


def recvall(sock: socket.socket, n: int) -> Optional[bytes]:
    """Receive exactly n bytes from socket."""
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def recv_msg(sock: socket.socket):
    """Receive a msgpack message from socket."""
    hdr = recvall(sock, 4)
    if not hdr:
        return None
    (length,) = struct.unpack("!I", hdr)
    body = recvall(sock, length)
    if body is None:
        return None
    return msgpack.unpackb(body, raw=False)


def send_msg(sock: socket.socket, obj: dict):
    """Send a msgpack message over socket."""
    body = msgpack.packb(obj, use_bin_type=True)
    sock.sendall(struct.pack("!I", len(body)) + body)


def make_interpreter():
    """Create TensorFlow Lite interpreter for the segmentation model."""
    import os
    
    if USE_TPU:
        try:
            # Try importing pycoral for TPU support
            from pycoral.utils import edgetpu
            tpus = edgetpu.list_edge_tpus()
            if tpus:
                if not os.path.exists(MODEL_TPU):
                    print(f"[coral_server] WARNING: TPU model not found at {MODEL_TPU}", flush=True)
                else:
                    print("[coral_server] loading DeepLab on EdgeTPU...", flush=True)
                    return edgetpu.make_interpreter(MODEL_TPU)
            print("[coral_server] USE_TPU=True but no TPU found -> CPU fallback", flush=True)
        except ImportError:
            print("[coral_server] pycoral not installed -> CPU fallback", flush=True)

    print("[coral_server] loading DeepLab on CPU...", flush=True)
    
    if not os.path.exists(MODEL_CPU):
        raise FileNotFoundError(
            f"\nModel file not found: {MODEL_CPU}\n"
            "Please ensure the model file exists at the configured path.\n"
            "You can customize MODEL_CPU path in coral_server_tflite.py"
        )
    
    return tflite.Interpreter(model_path=MODEL_CPU, num_threads=4)


def get_input_size(interpreter):
    """Get input tensor dimensions."""
    input_details = interpreter.get_input_details()
    input_shape = input_details[0]['shape']
    return input_shape[2], input_shape[1]  # (width, height)


def get_output(interpreter):
    """Get segmentation output from model."""
    output_details = interpreter.get_output_details()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return output_data


def main():
    """Main server loop."""
    interpreter = make_interpreter()
    interpreter.allocate_tensors()
    
    in_w, in_h = get_input_size(interpreter)
    print(f"[coral_server] OK. input: {in_w}x{in_h}. Listening on {HOST}:{PORT}", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)

    while True:
        conn, _ = srv.accept()
        conn.settimeout(CONN_TIMEOUT_S)

        try:
            req = recv_msg(conn)
            if not req:
                continue

            jpg = req.get("img_jpg", b"")
            bgr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                send_msg(conn, {"ok": False, "err": "bad jpeg"})
                continue

            # Downscale early for speed (VM-friendly)
            oh, ow = bgr.shape[:2]
            max_side = max(oh, ow)
            if max_side > MAX_SIDE_BEFORE_MODEL_RESIZE:
                scale = float(MAX_SIDE_BEFORE_MODEL_RESIZE) / float(max_side)
                bgr = cv2.resize(
                    bgr,
                    (max(1, int(ow * scale)), max(1, int(oh * scale))),
                    interpolation=cv2.INTER_AREA,
                )

            # DeepLab expects RGB
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rgb_in = cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_LINEAR)

            # Get input tensor details to check dtype
            input_details = interpreter.get_input_details()
            input_dtype = input_details[0]['dtype']
            
            # Convert to appropriate dtype for the model
            if input_dtype == np.uint8:
                # Quantized model expects UINT8 [0, 255]
                rgb_in = rgb_in.astype(np.uint8)
            else:
                # Float model expects normalized [0, 1]
                rgb_in = rgb_in.astype(np.float32) / 255.0

            # Set input tensor
            interpreter.set_tensor(input_details[0]['index'], np.expand_dims(rgb_in, axis=0))
            
            # Run inference
            interpreter.invoke()

            # Get output
            out = get_output(interpreter)
            if out.ndim == 4:
                out = out[0]  # Remove batch dimension
            if out.ndim == 3:
                out = np.argmax(out, axis=-1)
            mask = out.astype(np.uint8)

            person_pixels = int(np.sum(mask == PERSON_CLASS_ID))
            total_pixels = int(mask.size)
            area_ratio = float(person_pixels) / float(total_pixels) if total_pixels else 0.0

            if person_pixels > 0:
                ys, xs = np.where(mask == PERSON_CLASS_ID)
                cx_norm = float(np.mean(xs) / float(mask.shape[1]))  # 0..1
                cy_norm = float(np.mean(ys) / float(mask.shape[0]))  # 0..1
                person = True
            else:
                cx_norm, cy_norm = 0.5, 0.5
                person = False

            # Reply with compact features (no mask transfer)
            send_msg(conn, {
                "ok": True,
                "person": person,
                "cx": cx_norm,
                "cy": cy_norm,
                "area": area_ratio
            })

        except Exception as e:
            try:
                send_msg(conn, {"ok": False, "err": str(e)})
            except Exception:
                pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
