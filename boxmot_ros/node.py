import rclpy
import time
import numpy as np

from pathlib import Path

from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from detection_msgs.msg import Detections

from cv_bridge import CvBridge

from ultralytics import YOLO
from boxmot import BoTSORT, DeepOCSORT, OCSORT, HybridSORT, BYTETracker, StrongSORT

class BoxmotROS(Node):

    def __init__(self):
        super().__init__('boxmot_ros')

        self.declare_parameter("yolo_model",                "yolov8n.pt")
        self.declare_parameter("tracking_model",            "deepocsort")
        self.declare_parameter("reid_model",                "osnet_x0_25_msmt17.pt")
        self.declare_parameter("input_rgb_topic",           "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic",         "/camera/depth/points")
        self.declare_parameter("subscribe_depth",           False)
        self.declare_parameter("publish_annotated_image",   False)
        self.declare_parameter("rgb_topic",                 "/boxmot_ros/rgb_image")
        self.declare_parameter("depth_topic",               "/boxmot_ros/depth_image")
        self.declare_parameter("annotated_topic",           "/boxmot_ros/annotated_image")
        self.declare_parameter("detailed_topic",            "/boxmot_ros/tracking_result")
        self.declare_parameter("threshold",                 0.25)
        self.declare_parameter("device",                    "cpu")
        
        self.yolo_model                 = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.tracking_model             = self.get_parameter("tracking_model").get_parameter_value().string_value
        self.reid_model                 = self.get_parameter("reid_model").get_parameter_value().string_value
        self.input_rgb_topic            = self.get_parameter("input_rgb_topic").get_parameter_value().string_value
        self.input_depth_topic          = self.get_parameter("input_depth_topic").get_parameter_value().string_value
        self.subscribe_depth            = self.get_parameter("subscribe_depth").get_parameter_value().bool_value
        self.publish_annotated_image    = self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        self.rgb_topic                  = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic                = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.annotated_topic            = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.detailed_topic             = self.get_parameter("detailed_topic").get_parameter_value().string_value
        self.threshold                  = self.get_parameter("threshold").get_parameter_value().double_value
        self.device                     = self.get_parameter("device").get_parameter_value().string_value

        self.bridge = CvBridge()

        self.model = YOLO(self.yolo_model)
        self.model.fuse()

        if self.tracking_model == "deepocsort":
            self.tracker = DeepOCSORT(  model_weights=Path(self.reid_model), # which ReID model to use
                                        device=self.device,
                                        fp16=False )
            
        elif self.tracking_model == "strongsort":
            self.tracker = StrongSORT(  model_weights=Path(self.reid_model), # which ReID model to use
                                        device=self.device,
                                        fp16=False )
            
        elif self.tracking_model == "ocsort":
            self.tracker = OCSORT()

        elif self.tracking_model == "bytetrack":
            self.tracker = BYTETracker()

        elif self.tracking_model == "botsort":
            self.tracker = BoTSORT( model_weights=Path(self.reid_model), # which ReID model to use
                                    device=self.device,
                                    fp16=False )
            
        elif self.tracking_model == "hybridsort":
            self.tracker = HybridSORT(  model_weights=Path(self.reid_model), # which ReID model to use
                                        device=self.device,
                                        half=False,
                                        det_thresh=self.threshold )
        else:
            self.get_logger().error('Please select a valid value for "tracking_model" parameter')


        self.subscriber_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                 history=QoSHistoryPolicy.KEEP_LAST,
                                                 depth=1 )

        if self.subscribe_depth:
            self.rgb_message_filter     = Subscriber(self, Image, self.input_rgb_topic, qos_profile=self.subscriber_qos_profile)
            self.depth_message_filter   = Subscriber(self, PointCloud2, self.input_depth_topic, qos_profile=self.subscriber_qos_profile)

            self.synchornizer = ApproximateTimeSynchronizer([self.rgb_message_filter, self.depth_message_filter], 10, 1)
            self.synchornizer.registerCallback(self.sync_callback)

            self.publisher_depth  = self.create_publisher(PointCloud2, self.depth_topic, 10)

        else:
            self.subscription = self.create_subscription(Image, self.input_rgb_topic, self.image_callback, qos_profile=self.subscriber_qos_profile)
        
        self.publisher_results  = self.create_publisher(Detections, self.detailed_topic, 10)
        self.publisher_rgb      = self.create_publisher(Image, self.rgb_topic, 10)

        if self.publish_annotated_image:
            self.publisher_image    = self.create_publisher(Image, self.annotated_topic, 10)

        self.subscription  # prevent unused variable warning
        self.counter = 0
        self.time = 0

        self.tracking_msg = Detections()
        self.class_list_set = False


    def sync_callback(self, rgb_msg, depth_msg):
        start = time.time_ns()

        self.input_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        self.result = self.model.predict(source = self.input_image,
                                         conf=self.threshold,
                                         device=self.device,
                                         verbose=False)
        
        if (not self.class_list_set) and (self.result is not None):
            for i in range(len(self.result[0].names)):
                self.tracking_msg.full_class_list.append(self.result[0].names.get(i))
                self.class_list_set = True

        if self.result is not None:
            detection_list = []

            for bbox, cls, conf in zip(self.result[0].boxes.xywh, self.result[0].boxes.cls, self.result[0].boxes.conf):
                detection = []

                cx   = int(bbox[0])
                cy   = int(bbox[1])
                sw   = int(bbox[2])
                sh   = int(bbox[3])

                x1 = cx - (sw/2)
                y1 = cy - (sh/2)
                x2 = cx + (sw/2)
                y2 = cy + (sh/2)

                detection = [x1, y1, x2, y2, float(conf), int(cls)]

                detection_list.append(detection)

            detection_numpy = np.array(detection_list)
        else:
            detection_numpy = np.empty((0, 5))

        #  input is of shape  (x, y, x, y, conf, cls)
        #  output is of shape (x, y, x, y, id, conf, cls, ind)

        self.result_tracks = self.tracker.update(detection_numpy, self.input_image)

        if self.result_tracks is not None:
            
            self.tracking_msg.header            = rgb_msg.header

            for track in self.result_tracks:
                x1          = track[0].astype('int')
                y1          = track[1].astype('int')
                x2          = track[2].astype('int')
                y2          = track[3].astype('int')

                tracking_id = track[4].astype('int')
                confidence  = track[5].astype('float')
                class_id    = track[6].astype('int')

                cx = (x2 + x1)/2
                cy = (y2 + y1)/2
                sw = x2 - x1
                sh = y2 - y1

                self.tracking_msg.bbx_center_x.append(int(cx))
                self.tracking_msg.bbx_center_y.append(int(cy))
                self.tracking_msg.bbx_size_w.append(int(cx))
                self.tracking_msg.bbx_size_h.append(int(cx))
                self.tracking_msg.class_id.append(class_id)
                self.tracking_msg.tracking_id.append(tracking_id)
                self.tracking_msg.confidence.append(confidence)

            self.publisher_results.publish(self.tracking_msg)
            self.publisher_rgb.publish(rgb_msg)
            self.publisher_depth.publish(depth_msg)

            if self.publish_annotated_image:
                self.output_image = self.tracker.plot_results(self.input_image, show_trajectories=True)
                result_msg        = self.bridge.cv2_to_imgmsg(self.output_image, encoding="bgr8")
                self.publisher_image.publish(result_msg)

        self.counter += 1
        self.time += time.time_ns() - start

        if (self.counter == 100):
            self.get_logger().info('Callback execution time for 100 loops: %d ms' % ((self.time/100)/1000000))
            self.time = 0
            self.counter = 0


    def image_callback(self, rgb_image):
        start = time.time_ns()

        self.input_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="bgr8")

        self.result = self.model.predict(source = self.input_image,
                                         conf=self.threshold,
                                         device=self.device,
                                         verbose=False)
        
        if (not self.class_list_set) and (self.result is not None):
            for i in range(len(self.result[0].names)):
                self.tracking_msg.full_class_list.append(self.result[0].names.get(i))
                self.class_list_set = True
                
        if self.result is not None:
            detection_list = []

            for bbox, cls, conf in zip(self.result[0].boxes.xywh, self.result[0].boxes.cls, self.result[0].boxes.conf):
                detection = []

                cx   = int(bbox[0])
                cy   = int(bbox[1])
                sw   = int(bbox[2])
                sh   = int(bbox[3])

                x1 = cx - (sw/2)
                y1 = cy - (sh/2)
                x2 = cx + (sw/2)
                y2 = cy + (sh/2)

                detection = [x1, y1, x2, y2, float(conf), int(cls)]

                detection_list.append(detection)

            detection_numpy = np.array(detection_list)
        else:
            detection_numpy = np.empty((0, 5))

        #  input is of shape  (x, y, x, y, conf, cls)
        #  output is of shape (x, y, x, y, id, conf, cls, ind)

        self.result_tracks = self.tracker.update(detection_numpy, self.input_image)

        if self.result_tracks is not None:
            
            self.tracking_msg.header            = rgb_image.header

            for track in self.result_tracks:

                x1          = track[0].astype('int')
                y1          = track[1].astype('int')
                x2          = track[2].astype('int')
                y2          = track[3].astype('int')

                tracking_id = track[4].astype('int')
                confidence  = track[5].astype('float')
                class_id    = track[6].astype('int')

                cx = (x2 + x1)/2
                cy = (y2 + y1)/2
                sw = x2 - x1
                sh = y2 - y1

                self.tracking_msg.bbx_center_x.append(int(cx))
                self.tracking_msg.bbx_center_y.append(int(cy))
                self.tracking_msg.bbx_size_w.append(int(cx))
                self.tracking_msg.bbx_size_h.append(int(cx))
                self.tracking_msg.class_id.append(class_id)
                self.tracking_msg.tracking_id.append(tracking_id)
                self.tracking_msg.confidence.append(confidence)

            self.publisher_results.publish(self.tracking_msg)
            self.publisher_rgb.publish(rgb_image)

            if self.publish_annotated_image:
                self.output_image = self.tracker.plot_results(self.input_image, show_trajectories=True)
                result_msg        = self.bridge.cv2_to_imgmsg(self.output_image, encoding="bgr8")
                
                self.publisher_image.publish(result_msg)

        self.counter += 1
        self.time += time.time_ns() - start

        if (self.counter == 100):
            self.get_logger().info('Callback execution time for 100 loops: %d ms' % ((self.time/100)/1000000))
            self.time = 0
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    node = BoxmotROS()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()