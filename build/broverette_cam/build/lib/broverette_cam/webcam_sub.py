import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Update to the correct topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')

        # Determine the correct encoding and convert the image
        try:
            if msg.encoding == "yuv422":
                # Convert YUV422 to BGR
                yuv_image = self.br.imgmsg_to_cv2(msg, "mono8")
                current_frame = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_Y422)
            else:
                # Convert the image using the appropriate encoding
                current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            
            # Display image
            cv2.imshow("Camera Feed", current_frame)
            
            # Quit the window with 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except cv2.error as e:
            self.get_logger().error(f"OpenCV error: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    image_subscriber.destroy_node()
    rclpy.shutdown()
    # Close the OpenCV window
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
