import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import numpy as np
import cv2
import onnxruntime as ort
import os

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # --- Parameters 
        self.declare_parameter('model_name', 'yolov5_n_widerface_01')
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_hw', [640, 640])
        self.declare_parameter('use_trt', True)
        self.declare_parameter('assumed_face_width', 0.14)          # average adult face width (m)

        model_name          = self.get_parameter('model_name').value
        input_topic         = self.get_parameter('input_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold  = self.get_parameter('nms_threshold').value
        input_hw            = self.get_parameter('input_hw').value
        use_trt             = self.get_parameter('use_trt').value
        self.assumed_face_width = self.get_parameter('assumed_face_width').value
        self.input_h, self.input_w = input_hw

        # --- Locate the ONNX model
        pkg_dir = get_package_share_directory('x1_visual')
        model_path = os.path.join(pkg_dir, 'models', 'face_detection', f'{model_name}.onnx')

        # --- Build ONNXRuntime session with TRT or CUDA execution provider
        self.session = self._load_session(model_path, use_trt)

        # --- Cache input/output binding names for the session
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_name = self.session.get_outputs()[0].name
        self.output_shape = self.session.get_outputs()[0].shape

        self.get_logger().info(
            f'Face detection node ready \n'
            f'  model: {model_name} \n'
            f'  input: {self.input_name} {self.input_shape} \n'
            f'  output: {self.output_name} {self.output_shape} \n'
            f'  provider: {self.session.get_providers()}'
        )

        # --- ROS2 interfaces
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile = qos
        )

        self.camera_info: CameraInfo | None = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.crop_pub = self.create_publisher(
            Image, 
            '/face_crop', 
            qos_profile=qos
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/face_detection',
            qos_profile = qos
        )

        self.face_pose_pub = self.create_publisher(
            PoseStamped,
            '/face_pose',
            10
        )

        # --- Declare variables
        self.face_x_smooth = 0.0
        self.face_y_smooth = 0.0
        self.face_z_smooth = 0.0
        self.smooth_alpha = 0.9

    def _load_session(self, model_path: str, use_trt: bool) -> ort.InferenceSession:
        "Build an ONNXRuntime Inference Session with TensorRT (by default) or CUDA EP"

        if use_trt:
            providers = [
                ('TensorrtExecutionProvider', {
                    'device_id':                        0,
                    'trt_max_workspace_size':           512 * 1024 * 1024,
                    'trt_fp16_enable':                  True,
                    'trt_engine_cache_enable':          True,
                    'trt_engine_cache_path':            '/X1_ROS2_ws/src/x1_visual/models/face_detection',
                    'trt_force_sequential_engine_build': False
                }),
                ('CUDAExecutionProvider', {'device_id': 0}),
                'CPUExecutionProvider'
            ]
        else:
            providers = [
                ('CUDAExecutionProvider', {'device_id': 0}),
                'CPUExecutionProvider'
            ]

        session_options = ort.SessionOptions()
        session_options.log_severity_level = 3

        session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=providers
        )
        
        return session

    def _preprocess(self, cv_image: np.ndarray) -> np.ndarray:
        '''
        Resize, normalize, convert from BGR to RGB, and convert from
        (height, width, channel) to (channel, height, width)
        Return shape (1, 3, H, W) for ORT inference
        '''
        img = cv2.resize(cv_image, (self.input_w, self.input_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # color conversion from CV2's
        img = img.astype(np.float32) / 255.0        # pixel value normalization
        img = img.transpose(2, 0, 1)                # (H,W,C) => (C,H,W)
        img = np.ascontiguousarray(img)             
        img = np.expand_dims(img, axis=0)           # add batch dimension
        
        return img

    def _postprocess(self, raw_output: np.ndarray, image_h: int, image_w: int):
        '''
        Decode raw YOLOv5 output

        Args:
            raw_ouput: shape (1, 5, 8400) representing the (batch, [x,y,w,h,conf], anchor)
            image_h: original image height before preprocessing
            image_w: original image width before preprocessing
        
        Returns:
            list of ((x1, y1, w, h), confidence) tuples in original image coordinations
        '''

        detections = raw_output[0].T    # reshape from (5,8400)=>(8400,5) to iterate through anchors

        boxes = []
        confidences = []

        for det in detections:
            x_c, y_c, w, h, conf = det

            if conf < self.conf_threshold:
                continue

            # Scale coordinates from model input space back to original image dimensions
            x_c = x_c / self.input_w * image_w
            y_c = y_c / self.input_h * image_h
            w = w / self.input_w * image_w
            h = h / self.input_h * image_h

            x1 = int(x_c - w/2)
            y1 = int(y_c - h/2)
            boxes.append([x1, y1, int(w), int(h)])
            confidences.append(float(conf))
        
        if len(boxes) == 0:
            return []
        
        # Remove overlapping boxes for the same face
        indices = cv2.dnn.NMSBoxes(
            boxes,
            confidences,
            self.conf_threshold,
            self.nms_threshold
        )

        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                results.append((boxes[i], confidences[i]))

        return results

    def _select_primary_face(self, detections, image_h, image_w):
        '''
        Select the primary face in the scene by considering bbox size and detection confidence
        
        Args:
            detections: list of ((x1, y1, w, h), confidence) tuple of bboxes
            image_h, image_w: dimensions of the original image for normalization
        
        Returns:
            One primary detection tuple
        '''
        if not detections:
            return None
        if len(detections) == 1:
            return detections[0]
        
        image_area = image_h * image_w

        def score(detection):
            (x1, y1, w, h), conf = detection
            area_norm = w * h /image_area
            return 0.4 * area_norm + 0.6 * conf
        
        return max(detections, key=score)

    def _estimate_face_pose(self, bbox_cx: float, bbox_cy: float, bbox_w: float, stamp) -> PoseStamped | None:
        '''
        Monocular depth estimate of the 3D position of the face in the camera
        optical frame using the pinhole camera model and an assumed physical 
        face width.
        
        The camera optical frame convention:
            X: right
            Y: down
            Z: forward (into the scene)
        '''
        fx_camera = self.camera_info.k[0]
        fy_camera = self.camera_info.k[4]
        cx_camera = self.camera_info.k[2]
        cy_camera = self.camera_info.k[5]

        if bbox_w <= 0:
            return None

        z = self.assumed_face_width * fx_camera / bbox_w
        x = (bbox_cx - cx_camera) / fx_camera * z
        y = (bbox_cy - cy_camera) / fy_camera * z

        self.face_x_smooth = self.smooth_alpha * self.face_x_smooth + (1-self.smooth_alpha) * x if self.face_x_smooth else x
        self.face_y_smooth = self.smooth_alpha * self.face_y_smooth + (1-self.smooth_alpha) * y if self.face_y_smooth else y
        self.face_z_smooth = self.smooth_alpha * self.face_z_smooth + (1-self.smooth_alpha) * z if self.face_z_smooth else z

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'camera_color_optical_frame'
        pose.pose.position.x     = float(self.face_x_smooth)
        pose.pose.position.y     = float(self.face_y_smooth)
        pose.pose.position.z     = float(self.face_z_smooth)
        pose.pose.orientation.w  = 1.0     # identity — orientation not estimated

        return pose

    def image_callback(self, msg: Image):
        # --- Wait for camera info
        if self.camera_info is None:
            self.get_logger().info('Waiting for camera information')
            return
        
        fx_camera = self.camera_info.k[0]
        cx_camera = self.camera_info.k[2]

        # --- Convert ROS image message to OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_h, image_w = cv_image.shape[:2]

        # --- DEBUG - confirm images are arriving
        self.get_logger().debug(f'Image received: {image_w}x{image_h}')

        # --- Preprocess the image => (1, 3, H, W) float32
        img = self._preprocess(cv_image)

        # DEBUG 2 — confirm preprocessed shape and value range
        self.get_logger().debug(
            f'Preprocessed: shape={img.shape} '
            f'min={img.min():.3f} max={img.max():.3f}'
        )

        # --- Run the inference using ORT
        raw_output = self.session.run([self.output_name], {self.input_name: img})[0]

        # --- Postprocess to decode the boxes and apply non-maximal suppression (NMS)
        detections = self._postprocess(raw_output, image_h, image_w)

        primary_face = self._select_primary_face(detections, image_h, image_w)

        if primary_face is None:
            return
        
        # --- Crop the primary face and publish
        (x1, y1, w, h), _ = primary_face


        x1c = max(x1, 0)
        y1c = max(y1, 0)
        x2c = min(image_w, x1 + w)
        y2c = min(image_h, y1 + h)

        crop = cv_image[y1c:y2c, x1c:x2c]
        if crop.size > 0:
            crop_msg = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
            crop_msg.header = msg.header
            self.crop_pub.publish(crop_msg)
            # crop_w = x2c - x1c; crop_h = y2c - y1c
            # self.get_logger().info(f'Published a cropped image ({crop_w:4d},{crop_h:4d})')

        # --- Package into the Dection2DArray and publish
        det_array_msg = Detection2DArray()
        det_array_msg.header = msg.header
        
        bbox_cx = float(x1 + w/2)
        bbox_cy = float(y1 + h/2)
        det = Detection2D()
        det.bbox.center.position.x = bbox_cx
        det.bbox.center.position.y = bbox_cy
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        det_array_msg.detections.append(det)

        self.detection_pub.publish(det_array_msg)
        self.get_logger().info(f'Published {len(detections)} face detections')

        # --- Estimate face pose and publish
        pose = self._estimate_face_pose(bbox_cx=bbox_cx, bbox_cy=bbox_cy, bbox_w=w, stamp=msg.header.stamp)
        if pose is not None:
            self.face_pose_pub.publish(pose)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

def main():

    rclpy.init()
    node = FaceDetectionNode()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()