import tensorflow as tf
import cv2
import numpy as np
import sys
import os

print("--- TensorFlow GPU Verification ---")
print(f"TensorFlow Version: {tf.__version__}")
print(f"Python Version: {sys.version}")

gpu_devices = tf.config.list_physical_devices('GPU')

if not gpu_devices:
    print("\nERROR: No GPU detected by TensorFlow.")
    print("Please ensure the container was started with '--runtime nvidia'.")
    sys.exit(1)

print(f"\nSUCCESS: Found {len(gpu_devices)} GPU(s):")
for device in gpu_devices:
    try:
        details = tf.config.experimental.get_device_details(device)
        print(f"  - {device.name}, Name: {details.get('device_name', 'N/A')}")
    except Exception as e:
        print(f"  - Could not get details for device {device.name}: {e}")

try:
    print("\n--- Performing a simple GPU computation... ---")
    with tf.device('/GPU:0'):
        a = tf.constant([[1.0, 2.0], [3.0, 4.0]], dtype=tf.float32)
        b = tf.constant([[1.0, 1.0], [0.0, 1.0]], dtype=tf.float32)
        c = tf.matmul(a, b)
    
    print("SUCCESS: GPU computation test passed.")
    print("Result of matrix multiplication on GPU:")
    print(c.numpy())
except Exception as e:
    print(f"\nERROR: GPU computation test failed: {e}")
    sys.exit(1)

print("\n--- OpenCV Verification ---")
try:
    print(f"OpenCV Version: {cv2.__version__}")
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(img, (10, 10), (90, 90), (255, 0, 0), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if gray.shape != (100, 100):
        raise ValueError("Incorrect grayscale shape.")
    print("SUCCESS: OpenCV basic operations passed.")
except Exception as e:
    print(f"ERROR: OpenCV verification failed: {e}")
    sys.exit(1)

print("\n--- All Verifications Passed ---")
sys.exit(0)
