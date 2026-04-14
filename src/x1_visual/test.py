import onnxruntime as ort
import numpy as np

sess = ort.InferenceSession(
    '/X1_ROS2_ws/src/x1_visual/models/face_detection/yolov5_n_widerface_01.onnx',
    providers=[
        ('TensorrtExecutionProvider', {
            'device_id': 0,
            'trt_fp16_enable': True,
            'trt_max_workspace_size': 512 * 1024 * 1024,
            'trt_engine_cache_enable': True,
            'trt_engine_cache_path': '/X1_ROS2_ws/src/x1_visual/models/face_detection/',
        }),
        'CUDAExecutionProvider',
        'CPUExecutionProvider'
    ]
)

dummy = np.zeros((1, 3, 640, 640), dtype=np.float32)
out = sess.run(None, {'images': dummy})
print('Output shape:', out[0].shape)
print('Success')