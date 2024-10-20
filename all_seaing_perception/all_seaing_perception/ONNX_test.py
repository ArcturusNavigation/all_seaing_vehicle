#!/usr/bin/env python3
import onnxruntime as ort
import numpy as np
from PIL import Image
import cv2  # or any other library for image processing

# Load the ONNX model
onnx_model_path = '/home/arcturus/dev_ws/src/all_seaing_vehicle/all_seaing_perception/all_seaing_perception/models/yolov8m.onnx'  # Replace with the path to your ONNX file
session = ort.InferenceSession(onnx_model_path)

# Prepare the input (replace with the actual input size and model requirements)
input_image_path = 'your_image.jpg'  # Path to the image you want to run inference on
input_height, input_width = 480, 640  # The size used when exporting the model

# Load and preprocess the image
image = Image.open(input_image_path)
image = image.resize((input_width, input_height))  # Resize to match model input
image = np.array(image).astype(np.float32)  # Convert to numpy array and float32

# Normalization (if your model was trained with normalized images)
image = image / 255.0  # Normalize pixel values to [0, 1]

# Convert the image to the required format (batch size, channels, height, width)
input_tensor = np.transpose(image, (2, 0, 1))  # Convert from HWC to CHW format
input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch dimension

# Run inference
input_name = session.get_inputs()[0].name  # Get the input name for the model
outputs = session.run(None, {input_name: input_tensor})  # Run inference

# Process the output (depends on the model output format)
output = outputs[0]  # Assuming the output is the first element
print("Model output:", output)

# Further processing (e.g., post-processing for object detection, drawing bounding boxes, etc.)