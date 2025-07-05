import tensorflow as tf
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
    print("\n--- Verification Complete ---")

except Exception as e:
    print(f"\nERROR: GPU computation test failed: {e}")
    sys.exit(1)

sys.exit(0)
