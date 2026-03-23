import os
import torch
from ultralytics import YOLO

# Force PyTorch to use the system's native TensorRT paths
os.environ["CUDA_MODULE_LOADING"] = "LAZY"

# Load your model
model = YOLO('model.pt')

# Export with native settings
try:
    print("Starting export to engine...")
    model.export(
        format='engine',
        device='0',
        half=True,       # Use FP16 for Jetson Orin
        workspace=4,     # Give it 4GB of RAM
        simplify=True    # This helps avoid ONNX opset errors
    )
    print("SUCCESS: Engine built with embedded metadata.")
except Exception as e:
    print(f"FAILED: {e}")
