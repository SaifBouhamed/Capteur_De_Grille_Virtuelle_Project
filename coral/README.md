# Coral Server

A TensorFlow Lite semantic segmentation server for real-time person detection using Coral TPU or CPU fallback.

## Overview

The Coral Server runs a DeepLab segmentation model to detect persons in images. It supports:
- **Coral TPU Accelerator**: For hardware-accelerated inference (optional)
- **CPU Inference**: Fallback mode when TPU is unavailable

The server listens on TCP port 9900 and accepts requests with JPEG images, returning person detection results.

## Model Setup

The server requires DeepLab v3 MobileNet v2 PASCAL VOC model files:

### Model Files
- **CPU Model**: `/home/pinks/models/deeplabv3_mnv2_pascal_quant.tflite`
- **TPU Model**: `/home/pinks/models/deeplabv3_mnv2_pascal_quant_edgetpu.tflite` (optional)

### Downloading Models

**Option 1: Google Coral (Recommended)**
```bash
mkdir -p /home/pinks/models
cd /home/pinks/models

# Download CPU model
wget https://github.com/google-coral/test_data/raw/master/deeplabv3_mnv2_pascal_quant.tflite

# Download TPU model (optional)
wget https://github.com/google-coral/test_data/raw/master/deeplabv3_mnv2_pascal_quant_edgetpu.tflite
```

**Option 2: Custom Model Path**
Edit the server file to change the model paths:
```python
MODEL_CPU = "/your/custom/path/model.tflite"
MODEL_TPU = "/your/custom/path/model_edgetpu.tflite"
```

### Model Compatibility
- **Input**: 256×256 RGB image
- **Output**: Semantic segmentation (15 classes)
- **Class 15**: Person (PASCAL VOC)

## Model

## Server Responses

### Success Response
```json
{
  "ok": true,
  "person": true,
  "cx": 0.45,
  "cy": 0.52,
  "area": 0.15
}
```

- **person**: Boolean indicating if a person is detected
- **cx, cy**: Normalized center coordinates (0.0-1.0)
- **area**: Person pixels as ratio of total image area

### Error Response
```json
{
  "ok": false,
  "err": "error message"
}
```

## Quick Start

### Install & Run

```bash
./install_and_run.sh
```

This script will:
1. Check and install required Python packages
2. Set up the environment
3. Start the Coral Server on port 9900

### Dependencies

- Python 3.7+
- OpenCV
- NumPy
- MessagePack
- **TensorFlow Lite Runtime** (or TensorFlow as alternative)
- pycoral (optional, for full TPU support)

### Installation

The `install_and_run.sh` script automatically installs all dependencies:

```bash
./install_and_run.sh
```

**Note:** For Python 3.13+, `tflite-runtime` wheels are not available, so the script will install TensorFlow instead. TensorFlow includes TensorFlow Lite and provides full compatibility.

## Server Files

- `coral_server.py`: Full version with pycoral library (requires GPU/TPU support)
- `coral_server_tflite.py`: Simplified version using TensorFlow Lite Runtime directly (recommended)

## Configuration

Edit the server file to modify:
- `HOST`: Server bind address (default: `0.0.0.0`)
- `PORT`: Server port (default: `9900`)
- `MODEL_CPU`: Path to CPU model
- `MODEL_TPU`: Path to TPU model (if available)
- `USE_TPU`: Set to `True` to enable TPU (requires Coral USB Accelerator)
- `MAX_SIDE_BEFORE_MODEL_RESIZE`: Downscaling threshold (default: 640)

## Example Client

```python
import socket
import struct
import msgpack
import cv2

def send_request(image_path, host='localhost', port=9900):
    """Send an image to the Coral Server for inference."""
    img = cv2.imread(image_path)
    _, jpg = cv2.imencode('.jpg', img)
    
    msg = msgpack.packb({'img_jpg': jpg.tobytes()}, use_bin_type=True)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.sendall(struct.pack('!I', len(msg)) + msg)
    
    # Receive response
    hdr = sock.recv(4)
    length = struct.unpack('!I', hdr)[0]
    response = msgpack.unpackb(sock.recv(length), raw=False)
    sock.close()
    
    return response

result = send_request('test.jpg')
print(result)
```

## Troubleshooting

### Models not found
Make sure model files are at `/home/pinks/models/`

### TPU not detected
The server automatically falls back to CPU mode if no TPU is found. Set `USE_TPU=False` to disable TPU attempts.

### Import errors
Run `./install_and_run.sh` to ensure all dependencies are installed.

## License

Uses TensorFlow and Google Coral components.
