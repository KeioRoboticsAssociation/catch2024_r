import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

lower_green = np.array([80*180/360, 30, 50])
upper_green = np.array([240*180/360, 255, 200])

lower_red = np.array([0, 128, 50])
upper_red = np.array([1, 255, 255])

lower_red_2 = np.array([330*180/360, 128, 50])
upper_red_2 = np.array([360*180/360, 255, 255])

class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.publisher_ = self.create_publisher(String, 'setosio_cam', 10)
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.loop()
        
    def loop(self):
        cap = cv2.VideoCapture(0)
        # disable autofocus
        ret, frame = cap.read()
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
        # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) 
    
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 5000)
        cap.set(cv2.CAP_PROP_FOCUS, 100)
       
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        cap.set(cv2.CAP_PROP_GAIN, 20) 
        cap.set(cv2.CAP_PROP_EXPOSURE, 15)
        
        # cap.set(cv2.CAP_PROP_FPS, 15)

        while True:
            ret, frame = cap.read()
            # shrink image to 960x540
            # frame = cv2.resize(frame, (960, 540))
            if not ret:
                self.get_logger().info('No frame')
                break
            
            frame = frame[:,140:520]
            #コントラストを上げる
            frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=0.1)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gaus = cv2.GaussianBlur(gray, (15, 15), 5)
            _, bin = cv2.threshold(gaus, 100, 255, cv2.THRESH_BINARY)
            # bin = cv2.adaptiveThreshold(gaus, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 4)
            bin = cv2.morphologyEx(bin, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=3)
            cv2.imshow('bin', bin)
            # find contours
            contours, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            areas = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 5000:
                    continue
                areas.append(area)
                # max_area_index = np.argmax(areas)  
                # x, y, w, h = cv2.boundingRect(contours[max_area_index])
                x, y, w, h = cv2.boundingRect(contour)
                mask = np.zeros_like(frame)
                
                cv2.circle(mask, (x+int(w/2), y+int(h/2)), int((w+h)/4), (255, 255, 255), -1)
                masked_frame = cv2.bitwise_and(frame, mask)
                # convert masked frame to hsv
                hsv = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)
                # find green color
                green = cv2.inRange(hsv, lower_green, upper_green)
                red = cv2.inRange(hsv, lower_red, upper_red)
                red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
                red = cv2.bitwise_or(red, red_2)

                cv2.imshow('green', green)
                cv2.imshow('red', red)

                green_pixel_ratio = cv2.countNonZero(green)/cv2.countNonZero(cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY))
                red_pixel_ratio = cv2.countNonZero(red)/cv2.countNonZero(cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY))
                if green_pixel_ratio == 0:
                    green_pixel_ratio = 0.000001
                if red_pixel_ratio == 0:
                    red_pixel_ratio = 0.000001

                print(green_pixel_ratio, red_pixel_ratio)
                # find red color

 
                if green_pixel_ratio/red_pixel_ratio > 150 and green_pixel_ratio > 0.03:
                    cv2.putText(frame, 'norishio', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.circle(frame, (x+int(w/2), y+int(h/2)), int((w+h)/4), (0, 255, 0), 4)
                elif red_pixel_ratio/green_pixel_ratio >5 :
                    cv2.putText(frame, 'ebishio', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.circle(frame, (x+int(w/2), y+int(h/2)), int((w+h)/4), (0, 0, 255), 4)
                else:
                    cv2.putText(frame, 'yuzushio', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.circle(frame, (x+int(w/2), y+int(h/2)), int((w+h)/4), (0, 255, 255), 4)

                # cv2.putText(frame, str(area), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # print(areas)
            #biggest area index
            if not areas:
                self.get_logger().info('No contours')
            else:
                pass
            

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    image_node = ImageNode()

    rclpy.spin(image_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()