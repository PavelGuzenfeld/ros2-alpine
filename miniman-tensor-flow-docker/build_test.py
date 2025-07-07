import tensorflow as tf
import cv2
import numpy as np
import sys
import os

print("--- TensorFlow GPU Verification ---")
print(f"TensorFlow Version: {tf.__version__}")
print(f"Python Version: {sys.version}")

# Configure GPU memory growth to prevent OOM
try:
    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    if physical_devices:
        for device in physical_devices:
            tf.config.experimental.set_memory_growth(device, True)
        print("SUCCESS: GPU memory growth configured")
except Exception as e:
    print(f"WARNING: Could not configure GPU memory growth: {e}")

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
    
    # Use smaller matrices and limit memory usage for Jetson
    with tf.device('/GPU:0'):
        # Smaller test to avoid OOM on Jetson devices
        a = tf.constant([[1.0, 2.0]], dtype=tf.float32)  # Smaller matrix
        b = tf.constant([[1.0], [1.0]], dtype=tf.float32)  # Smaller matrix
        c = tf.matmul(a, b)
    
    print("SUCCESS: GPU computation test passed.")
    print("Result of matrix multiplication on GPU:")
    print(c.numpy())
    
    # Test memory info if available
    try:
        memory_info = tf.config.experimental.get_memory_info('GPU:0')
        print(f"GPU Memory - Current: {memory_info['current']//1024//1024} MB, Peak: {memory_info['peak']//1024//1024} MB")
    except Exception as e:
        print("GPU memory info not available")
        
except Exception as e:
    print(f"\nERROR: GPU computation test failed: {e}")
    print("This might be due to insufficient GPU memory on Jetson devices.")
    # Don't exit here, continue with other tests

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

print("\n--- GStreamer Support Verification ---")
try:
    import subprocess
    
    # Check GStreamer version
    try:
        result = subprocess.run(['gst-launch-1.0', '--version'], 
                               capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            version_line = result.stdout.strip().split('\n')[0]
            print(f"GStreamer Version: {version_line}")
            
            # Check if it's version 1.24.9
            if "1.24.9" in version_line:
                print("SUCCESS: Custom GStreamer 1.24.9 build detected!")
            else:
                print(f"INFO: GStreamer version detected but not 1.24.9: {version_line}")
        else:
            print("WARNING: Could not determine GStreamer version")
    except Exception as e:
        print(f"WARNING: Could not check GStreamer version: {e}")
    
    # Check Python GStreamer bindings (GObject Introspection)
    try:
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        
        Gst.init(None)
        gst_version = ".".join([str(Gst.VERSION_MAJOR), str(Gst.VERSION_MINOR), str(Gst.VERSION_MICRO)])
        print(f"SUCCESS: Python GStreamer bindings available! Version: {gst_version}")
        
        # Test creating a simple pipeline
        pipeline_str = "videotestsrc num-buffers=1 ! fakesink"
        pipeline = Gst.parse_launch(pipeline_str)
        if pipeline:
            print("SUCCESS: GStreamer Python pipeline creation test passed!")
        else:
            print("WARNING: Could not create GStreamer pipeline from Python")
            
    except ImportError as e:
        print(f"WARNING: Python GStreamer bindings not available: {e}")
    except Exception as e:
        print(f"WARNING: Python GStreamer test failed: {e}")
    
    # Check if OpenCV was built with GStreamer support
    build_info = cv2.getBuildInformation()
    gstreamer_support = "GStreamer:" in build_info and "YES" in build_info.split("GStreamer:")[1].split("\n")[0]
    
    if gstreamer_support:
        print("SUCCESS: OpenCV was built with GStreamer support.")
        # Try to extract GStreamer version from OpenCV build info
        try:
            gst_line = [line for line in build_info.split('\n') if 'GStreamer:' in line][0]
            print(f"OpenCV GStreamer info: {gst_line.strip()}")
        except:
            pass
    else:
        print("WARNING: OpenCV was not built with GStreamer support.")
    
    # Test VideoCapture backends
    backends = []
    backend_names = {cv2.CAP_GSTREAMER: "GStreamer", cv2.CAP_V4L2: "V4L2", cv2.CAP_FFMPEG: "FFmpeg"}
    
    for backend, name in backend_names.items():
        try:
            cap = cv2.VideoCapture()
            if cap.open(0, backend):
                backends.append(name)
                cap.release()
        except:
            pass
    
    if backends:
        print(f"SUCCESS: Available video backends: {', '.join(backends)}")
    else:
        print("INFO: No video backends available (this is normal in containers without camera access).")
        
    # Test GStreamer pipeline creation via OpenCV (if available)
    if cv2.CAP_GSTREAMER in [cv2.CAP_GSTREAMER] and gstreamer_support:
        try:
            # Test a simple GStreamer pipeline
            pipeline = "videotestsrc num-buffers=1 ! videoconvert ! appsink"
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print("SUCCESS: OpenCV GStreamer pipeline test passed!")
                else:
                    print("INFO: OpenCV GStreamer pipeline opened but no frame captured")
                cap.release()
            else:
                print("INFO: Could not open OpenCV GStreamer test pipeline")
        except Exception as e:
            print(f"INFO: OpenCV GStreamer pipeline test failed: {e}")
        
except Exception as e:
    print(f"WARNING: GStreamer verification encountered an issue: {e}")

print("\n--- System Information ---")
try:
    print(f"CUDA Runtime Version: {tf.sysconfig.get_build_info()['cuda_version']}")
    print(f"cuDNN Version: {tf.sysconfig.get_build_info()['cudnn_version']}")
except:
    print("Could not retrieve CUDA/cuDNN version info.")

print("\n--- All Core Verifications Completed ---")
