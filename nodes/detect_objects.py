#!/usr/bin/env python3

import cv2
import rclpy
import message_filters
import ros2_numpy
import threading
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ultralytics import YOLO, SAM
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from stretch_web_teleop_helpers.conversions import (
    cv2_image_to_ros_msg
)
from cv_bridge import CvBridge
from ultralytics.models.sam import Predictor as SAMPredictor
from ultralytics import SAM, FastSAM
from stretch_web_teleop.srv import DetectObject
from std_srvs.srv import SetBool
from vision_msgs.msg import Pose2D, Point2D, BoundingBox2D, BoundingBox2DArray

class ObjectDetectionNode(Node):
    def __init__(self, confidence_threshold=0.2):
        super().__init__("detect_objects")
        # Load the models
        # self.model = YOLO("yolov8s-worldv2.pt", verbose=False)
        # self.model = SAM("mobile_sam.pt")
        self.model = FastSAM("FastSAM-s.pt")
        self.detected_objects = []
        # self.model = YOLO("yolo11n-seg.pt")
        # overrides = dict(conf=0.1, task="segment", mode="predict", imgsz=640, model="mobile_sam.pt")
        # self.predictor = SAMPredictor(overrides=overrides)   
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
        self.confidence_threshold = confidence_threshold
        self.curr_box = None

        self.rgb_topic_name = '/camera/color/image_raw' #'/camera/infra1/image_rect_raw'
        self.rgb_image_subscriber = self.create_subscription(
            Image, 
            self.rgb_topic_name, 
            self.image_callabck, 
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.cv_bridge = CvBridge()
        self.visualize_object_detections_pub = self.create_publisher(CompressedImage, '/camera/color/image_with_bb/compressed', 1)

        self.detect_object_service = self.create_service(
            DetectObject, "detect_object", self.detect_objects_callback
        )

        self.feeding_mode = False
        self.feeding_mode_service = self.create_service(
            SetBool, "feeding_mode", self.feeding_mode_callback
        )

    def feeding_mode_callback(self, req, res):
        self.get_logger().info(f"Feeding mode service: {req.data}")
        self.feeding_mode = req.data
        res.success = True
        return res

    def get_pixels_in_radius(self, center, radius):
        x, y = center
        
        # Create a grid of offsets
        dx = np.arange(-radius, radius + 1)
        dy = np.arange(-radius, radius + 1)
        
        # Create a meshgrid of offsets
        dx, dy = np.meshgrid(dx, dy, indexing='ij')
        
        # Calculate the squared distance from the center
        distance_squared = dx**2 + dy**2
        
        # Get the coordinates that are within the specified radius
        mask = distance_squared <= radius**2
        pixels = np.column_stack((x + dx[mask], y + dy[mask]))
        
        return pixels

    def find_minimum_encapsulating_box_for_selected(selected_bbox):
        # Function to compute the area of a bounding box
        def area(bbox):
            x_min, y_min, x_max, y_max = bbox
            return (x_max - x_min) * (y_max - y_min)
        
        # Initialize the minimum area and box to be None
        min_area = float('inf')
        min_area_box = None
        
        # Iterate over the candidate bounding boxes
        for candidate in self.detected_objects:
            # Find the minimum encapsulating box for the selected_bbox and the candidate
            x_min = min(selected_bbox[0], candidate[0])
            y_min = min(selected_bbox[1], candidate[1])
            x_max = max(selected_bbox[2], candidate[2])
            y_max = max(selected_bbox[3], candidate[3])
            
            # Calculate the area of the encapsulating box
            encapsulating_box = [x_min, y_min, x_max, y_max]
            box_area = area(encapsulating_box)
            
            # If this box has a smaller area, update the result
            if box_area < min_area:
                min_area = box_area
                min_area_box = encapsulating_box
        
        return min_area_box
        
    def detect_objects_callback(self, req, res):
        rotated_image = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)
        self.detected_objects = []
        bboxes = []
        if req.data == True:
            object_detections = self.model(rotated_image, device='cpu', conf=0.75, iou=0.9, stream=True)
            self.detected_objects = []
            for detection in object_detections:
                objects = detection.boxes
                for _, box in enumerate(objects.xyxy):
                    x_min = int(box[0])
                    x_max = int(box[2])
                    y_min = int(box[1])
                    y_max = int(box[3])
                    self.detected_objects.append([x_min, y_min, x_max, y_max])

                    bbox = BoundingBox2D()
                    bbox.center.position.x = ((x_max + x_min)/2)/self.rgb_image.shape[0]
                    bbox.center.position.y = ((y_max + y_min)/2)/self.rgb_image.shape[1]
                    bbox.size_x = (x_max - x_min)/self.rgb_image.shape[0]
                    bbox.size_y = (y_max - y_min)/self.rgb_image.shape[1]
                    bboxes.append(bbox)

                    print([x_min, y_min, x_max, y_max], bbox.center.position.x, bbox.center.position.y, self.rgb_image.shape)
        res.boxes = bboxes
        return res
    
    def detect_object_callback(self, req, res):
        dw = dh = 10
        bbox = [[
            req.x - dw,
            req.y - dh,
            req.x + dw,
            req.y + dw
        ]]
        object_detections = self.model(self.rgb_image, points=[[req.y, req.x]], stream=True)
        # object_detections = self.model(self.rgb_image)
        for detection in object_detections:
            objects = detection.boxes
            curr_confidence = 0
            self.curr_box = None
            for _, (confidence, box) in enumerate(zip(objects.conf, objects.xyxy)):
                print(confidence, box)
                if confidence > curr_confidence:
                    res.x_min = int(box[0])
                    res.x_max = int(box[2])
                    res.y_min = int(box[1])
                    res.y_max = int(box[3])
                    self.curr_box = [res.x_min, res.y_min, res.x_max, res.y_max]
                curr_confidence = confidence
        return res
    
    def image_callabck(self, ros_rgb_image):
        self.rgb_image = ros2_numpy.numpify(ros_rgb_image)

        # OpenCV expects bgr images, but numpify by default returns rgb images.
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

        # if self.curr_box != None:
        #     self.draw_boxes(self.rgb_image, [self.curr_box])
        # msg = cv2_image_to_ros_msg(self.rgb_image, compress=True, bridge=self.cv_bridge)
        # msg.header.stamp = ros_rgb_image.header.stamp
        # if msg is not None:
        #     self.visualize_object_detections_pub.publish(msg)

        # Rotate the image by 90deg to account for camera
        # orientation. In the future, this may be performed at the
        # image source.

        rotated_image = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)

        self.draw_boxes(rotated_image, self.detected_objects)

        # detections_2d, output_image = self.apply_to_image(detection_box_image)

        output_image = cv2.rotate(rotated_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if output_image is not None:
            # output_image = ros2_numpy.msgify(Image, output_image, encoding='rgb8')
            msg = cv2_image_to_ros_msg(output_image, compress=True, bridge=self.cv_bridge)
            msg.header.stamp = ros_rgb_image.header.stamp
            if msg is not None:
                self.visualize_object_detections_pub.publish(msg)

    def apply_to_image(self, rgb_image, draw_output=False):
        x = 0
        y = 0
        # import IPython; IPython.embed()
        object_detections = self.model(rgb_image, device='cpu', conf=0.85, iou=0.9, stream=True)
        # object_detections = self.predictor(rgb_image, crop_n_layers=1, points_stride=64)
        # print(object_detections.boxes)
        results = []
        boxes = []
        # object_detections = self.model(self.rgb_image)
        for detection in object_detections:
            names = detection.names
            objects = detection.boxes
            for _, (cls, confidence, box) in enumerate(zip(objects.cls, objects.conf, objects.xyxy)):
            # confidence = detection
                # if confidence > 0.2:
                object_class_id = int(cls)
                class_label = names[object_class_id]
                # print(box)
                x_min = int(box[0])
                x_max = int(box[2])
                y_min = int(box[1])
                y_max = int(box[3])
                box = [x_min, y_min, x_max, y_max]

                # print(class_label, ' detected')
                boxes.append(box)
                results.append({'class_id': object_class_id,
                                'label': class_label,
                                'confidence': confidence,
                                'box': box})

        boxes = np.array(boxes)
        # print(boxes)
        # Get the min and max coordinates for easier comparisons
        # x_min = boxes[:, 0]
        # y_min = boxes[:, 1]
        # x_max = boxes[:, 2]
        # y_max = boxes[:, 3]
        
        # # Create a mask to filter out contained boxes
        # contained_mask = np.zeros(len(boxes), dtype=bool)
        
        # for i in range(len(boxes[1:])):
        #     # Check if box i is contained in any other box
        #     contained_mask[i] = np.any(
        #         (x_min[i] >= x_min) & (y_min[i] >= y_min) & 
        #         (x_max[i] <= x_max) & (y_max[i] <= y_max) & 
        #         (np.arange(len(boxes)) != i)  # Exclude the box itself
        #     )
            
        # pruned_boxes = boxes[~contained_mask]
        output_image = rgb_image.copy()
        # import IPython; IPython.embed()
        self.draw_boxes(output_image, boxes)
        # draw_output = True
        # output_image = None
        # if draw_output:
        #     output_image = rgb_image.copy()
        #     for detection_dict in results:
        #         self.draw_detection(output_image, detection_dict)

        return results, output_image

    def draw_boxes(self, image, boxes):
        color = (0, 0, 255)
        rectangle_line_thickness = 1
        for index, box in enumerate(boxes):
            x_min, y_min, x_max, y_max = box

            # Label for the bounding box
            label = str(index)
            
            # Get the size of the label text
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            # Define the position for the label (top-left corner of the bounding box)
            label_x = x_min
            label_y = y_min + text_height  # Position slightly below the top-left corner

            # Check if the text box is too small, if so, fill the bounding box
            if text_width < (x_max - x_min) / 2 or text_height < (y_max - y_min) / 2:
                label_x = label_x + 5 # Position slightly to the right
                # Create a mask for the semi-transparent rectangle and place behind the label
                overlay = image[y_min:y_min + text_height + 10, x_min:x_max]
                white_rect = np.ones(overlay.shape, dtype=np.uint8) * 255
                label_rect = cv2.addWeighted(overlay, 0.4, white_rect, 0.4, 1.0)  
                image[y_min:y_min + text_height + 10, x_min:x_max] = label_rect
                
                # Put the label
                cv2.putText(image, label, (label_x, label_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)  # Black text
            else:
                # Create a mask for the semi-transparent rectangle and place behind the label
                overlay = image[y_min:y_max, x_min:x_max]
                white_rect = np.ones(overlay.shape, dtype=np.uint8) * 255
                label_rect = cv2.addWeighted(overlay, 0.4, white_rect, 0.4, 1.0)  
                image[y_min:y_max, x_min:x_max] = label_rect
                # Draw the label normally
                cv2.putText(image, label, (label_x, label_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)  # Black text

            # Draw the bounding box
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, rectangle_line_thickness)

    def draw_detection(self, image, detection_dict):
        font_scale = 0.75
        line_color = [0, 0, 0]
        line_width = 1
        font = cv2.FONT_HERSHEY_PLAIN
        class_label = detection_dict['label']
        confidence = detection_dict['confidence']
        box = detection_dict['box']
        x_min, y_min, x_max, y_max = box
        output_string = '{0}, {1:.2f}'.format(class_label, confidence)
        color = (0, 0, 255)
        rectangle_line_thickness = 2 #1
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, rectangle_line_thickness)

        # see the following page for a helpful reference
        # https://stackoverflow.com/questions/51285616/opencvs-gettextsize-and-puttext-return-wrong-size-and-chop-letters-with-low

        label_background_border = 2
        (label_width, label_height), baseline = cv2.getTextSize(output_string, font, font_scale, line_width)
        label_x_min = x_min
        label_y_min = y_min
        label_x_max = x_min + (label_width + (2 * label_background_border))
        label_y_max = y_min + (label_height + baseline + (2 * label_background_border))

        text_x = label_x_min + label_background_border
        text_y = (label_y_min + label_height) + label_background_border

        # cv2.rectangle(image, (label_x_min, label_y_min), (label_x_max, label_y_max), (255, 255, 255), cv2.FILLED)
        # cv2.putText(image, output_string, (text_x, text_y), font, font_scale, line_color, line_width, cv2.LINE_AA)

def main():
    rclpy.init()
    detect_objects = ObjectDetectionNode()
    detect_objects.get_logger().info("Created!")

    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor()

    # Spin in the background, as the node initializes
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(detect_objects,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Spin in the foreground
    spin_thread.join()

    detect_objects.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()