#!/usr/bin/env python3

"""
A convenience node to visualize ROS2 compressed images using CV2.
"""
# Standard imports
import threading

# Third-party imports
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# Local imports


class CompressedImageVisualizer(Node):
    """
    A convenience node to visualize ROS2 compressed images using CV2.
    """

    def __init__(self):
        """
        Initialize the node.
        """
        super().__init__("compressed_image_visualizer")

        self.bridge = CvBridge()

        # Get the latest image
        self.latest_img_lock = threading.Lock()
        self.latest_img = None
        self.compressed_img_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.compressed_img_callback,
            1,
        )

        self.get_logger().info("Compressed Image Visualizer node initialized.")

    def compressed_img_callback(self, msg: CompressedImage) -> None:
        """
        Callback for the compressed image subscriber.
        """
        # Convert the compressed image to a CV2 image
        cv2_img = self.bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        with self.latest_img_lock:
            self.latest_img = cv2_img

    def run(self, rate_hz: int = 10) -> None:
        """
        Loop at the specified rate, and render the latest image in CV2.

        :param rate_hz: The rate at which to loop and render images.
        """
        rate = self.create_rate(rate_hz)

        while rclpy.ok():
            latest_img = None
            with self.latest_img_lock:
                latest_img = self.latest_img
                self.latest_img = None
            if latest_img is not None:
                cv2.imshow("Compressed Image", latest_img)
                cv2.waitKey(1)
            rate.sleep()

        cv2.destroyAllWindows()


def main(args=None):
    """
    Launch the ROS node and spin.
    """
    rclpy.init(args=args)

    node = CompressedImageVisualizer()
    executor = MultiThreadedExecutor(num_threads=2)
    # Spin in the background since rendering images will block
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Run face detection
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    # Terminate this node
    node.destroy_node()
    rclpy.shutdown()
    # Join the spin thread (so it is spinning in the main thread)
    spin_thread.join()


if __name__ == "__main__":
    main()
