import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import cv2
import numpy as np

class camera_sub(Node):

    def __init__(self):
        super().__init__('line_following_node')
        self.camera_sub = self.create_subscription(Image,'/rpi_video_feed',self.camera_cb,10)
        self.error_value_publisher = self.create_publisher(Int16, '/line_following_error', 10)
        self.error_msg = Int16()

        self.bridge=CvBridge()



    def camera_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'mono8')
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
    
        roi = frame[int(frame.shape[0]*0.5):frame.shape[0], 0:frame.shape[1]] 

        edged = cv2.Canny(roi, 90, 150) 

        lines = cv2.HoughLinesP(edged, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        mid_point_line = None

        

        if lines is not None:
            sorted_lines = sorted(lines, key=lambda line: line[0][0])
            center_x = frame.shape[1] // 2

            left_lines = [line for line in sorted_lines if line[0][0] < center_x]
            right_lines = [line for line in sorted_lines if line[0][0] > center_x]

            inner_left_line = min(left_lines, key=lambda line: abs(line[0][0] - center_x)) if left_lines else None
            inner_right_line = min(right_lines, key=lambda line: abs(line[0][0] - center_x)) if right_lines else None

            if inner_left_line is not None and inner_right_line is not None:
                for x1, y1, x2, y2 in [inner_left_line[0], inner_right_line[0]]:
                    cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)

                mid_point_line = (inner_left_line[0][0] + inner_right_line[0][0]) // 2
                cv2.circle(roi, (mid_point_line, roi.shape[0]//2), 5, (255, 0, 0), -1)

                error = center_x - mid_point_line
            else:
                error = 0  




        print(f"Calculated error: {error}, type: {type(error)}")
        self.error_msg.data = int(error)
        self.error_value_publisher.publish(self.error_msg)

        cv2.imshow('Frame', frame)
        cv2.imshow('Canny Output', edged)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)

    sensor_sub = camera_sub()

    rclpy.spin(sensor_sub)
    sensor_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


