import rclpy
import time
import numpy as np

from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from detection_msgs.msg import Detections

from cv_bridge import CvBridge

from boxmot import BoTSORT, DeepOCSORT, OCSORT, HybridSORT, BYTETracker, StrongSORT

class BoxmotROS(Node):

    def __init__(self):
        super().__init__('boxmot_ros')

        self.declare_parameter("tracking_model",            "deepocsort")
        self.declare_parameter("reid_model",                "osnet_x0_25_msmt17.pt")
        self.declare_parameter("input_topic",               "/input")
        self.declare_parameter("publish_annotated_image",   False)
        self.declare_parameter("annotated_topic",           "/boxmot_ros/annotated_image")
        self.declare_parameter("detailed_topic",            "/boxmot_ros/tracking_result")
        self.declare_parameter("threshold",                 0.25)
        self.declare_parameter("device",                    "cpu")
        
        self.tracking_model             = self.get_parameter("tracking_model").get_parameter_value().string_value
        self.reid_model                 = self.get_parameter("reid_model").get_parameter_value().string_value
        self.input_topic                = self.get_parameter("input_topic").get_parameter_value().string_value
        self.publish_annotated_image    = self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        self.annotated_topic            = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.detailed_topic             = self.get_parameter("detailed_topic").get_parameter_value().string_value
        self.threshold                  = self.get_parameter("threshold").get_parameter_value().double_value
        self.device                     = self.get_parameter("device").get_parameter_value().string_value

        self.bridge = CvBridge()

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

        self.subscription       = self.create_subscription(Detections, self.input_topic, self.image_callback, qos_profile=self.subscriber_qos_profile)
        
        self.publisher_results  = self.create_publisher(Detections, self.detailed_topic, 10)

        if self.publish_annotated_image:
            self.publisher_image    = self.create_publisher(Image, self.annotated_topic, 10)

        self.subscription  # prevent unused variable warning
        self.counter = 0
        self.time = 0

        self.tracking_msg = Detections()

    def image_callback(self, received_msg):
        start = time.time_ns()

        self.input_image = self.bridge.imgmsg_to_cv2(received_msg.source_rgb, desired_encoding="bgr8")

        if received_msg.detections:
            detection_list = []

            for i in range(len(received_msg.class_id)):
                detection = []

                clid = received_msg.class_id[i]
                conf = received_msg.confidence[i]

                cx   = received_msg.bbx_center_x[i]
                cy   = received_msg.bbx_center_y[i]
                sw   = received_msg.bbx_size_w[i]
                sh   = received_msg.bbx_size_h[i]

                x1 = cx - (sw/2)
                y1 = cy - (sh/2)
                x2 = cx + (sw/2)
                y2 = cy + (sh/2)

                detection = [x1, y1, x2, y2, conf, clid]

                detection_list.append(detection)

            detection_numpy = np.array(detection_list)
        else:
            detection_numpy = np.empty((0, 5))

        #  input is of shape (x, y, x, y, conf, cls)
        # output is of shape (x, y, x, y, id, conf, cls, ind)

        self.result_tracks = self.tracker.update(detection_numpy, self.input_image)

        if self.result_tracks is not None:
            
            self.tracking_msg.header            = received_msg.header
            self.tracking_msg.source_rgb        = received_msg.source_rgb
            self.tracking_msg.source_depth      = received_msg.source_depth
            self.tracking_msg.full_class_list   = received_msg.full_class_list

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