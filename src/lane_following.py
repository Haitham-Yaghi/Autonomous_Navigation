
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist

"""
def image_callback(msg):
    while(True):
        bridge = CvBridge()
        # Convert ROS image message to OpenCV image
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Apply image processing operations
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Convert OpenCV image to ROS image message
        gray_msg = bridge.cv2_to_imgmsg(gray_img, encoding='passthrough')
        # Publish the processed image
        cv2.imshow("Original Image", gray_img)
        cv2.waitKey(1)
        pub.publish(gray_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('image_processor')
    # Create a subscriber to the camera topic
    sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    # Create a publisher for the processed image
    pub = rospy.Publisher('/processed_image', Image, queue_size=10)
    # Spin the node to receive and process images
    rospy.spin()
"""
#global cv_image
# img=[]
class LaneDetector(object):

    def __init__(self):
        self.cv_image=None
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size = 1)
        self.speed_cmd = Twist()
        self.lastError = 0
       

        

    def camera_callback(self,data):
        #print(type(data))
        try:
            
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding = "bgr8")
            # cv2.imshow("hi",self.cv_image)
            # cv2.waitKey(1)
            #image_carrier(cv_image)
            # centroid_arr=[]

            result_img, centroid_arr = self.track_points_direction(self.cv_image)
                
            if len(centroid_arr) == 0:
                print("ERROR: no points detected!!! fix!")

                self.speed_cmd.linear.x = 0
                if self.lastError > 0:
                    self.speed_cmd.angular.z = -0.4
                elif self.lastError < 0:
                    self.speed_cmd.angular.z = 0.4
                else:
                    self.speed_cmd.angular.z = 0
                self.speed_pub.publish(self.speed_cmd)

            else:
                # print("is it you")
                follow_pt = self.get_point_to_follow(centroid_arr)
               # print("yes")
                print("This is follow point" , follow_pt)
                cv2.circle(result_img, follow_pt, 5, (0, 0, 255), -1) #draw red circle over the follow centroid
                # print("line 63")
                # cv2.imshow("result",result_img)
                # cv2.waitKey(1)

                error_x = follow_pt[0] - 343/2

                Kp = 0.003
                Kd = 0.007

                angular_z = Kp * error_x + Kd * (error_x - self.lastError)
                self.lastError = error_x

            
                self.speed_cmd.linear.x = min(((1 - abs(error_x) / (343/2)) ** 2.2), 0.5)
                self.speed_cmd.angular.z = -max(angular_z, -7.0) if angular_z < 0 else -min(angular_z, 7.0)
                self.speed_pub.publish(self.speed_cmd)
                

        except Exception as e:
            print(e)
        
        # return self.cv_image
    
    # def run(self):
    #     cv2.imshow("hi",img[-1])
    #     cv2.waitKey(1)
    

    def birds_eye_view(self, img, pt1, pt2, pt3, pt4):
        # print("hi there pt 5")
        h,w=img.shape[:2]

        input_pts = np.float32([[pt1[0],pt1[1]],
                                [pt2[0],pt2[1]],
                                [pt3[0],pt3[1]],
                                [pt4[0],pt4[1]]])

        output_pts = np.float32([[0, h],
                                [0, 0],
                                [w, 0],
                                [w, h]])

        # Compute the perspective transform M
        M = cv2.getPerspectiveTransform(input_pts,output_pts)          
        birds_eye_img = cv2.warpPerspective(img,M,(w, h),flags=cv2.INTER_LINEAR)

        # cv2.imshow("Bird's eye view", birds_eye_img)
        # cv2.waitKey(1)
        # print("hi there pt 6")
        return birds_eye_img

    def point_generator(self, pt1, pt2, h_i,  h34, width): #bottom two points and the height at which the top points are to be calculated
        # print("hi there pt 3")
        pt_i = (320 + (width//2), h_i) #intersection point

        slope1 = float((pt_i[1]-pt1[1]))/(pt_i[0]-pt1[0])
        w1 = int(pt_i[0] - ((pt_i[1]-h34)/slope1))

        slope2 = float((pt_i[1]-pt2[1]))/(pt_i[0]-pt2[0])
        w2 = int(pt_i[0] - ((pt_i[1]-h34)/slope2))

        # print((w1,h),(w2,h))
        # print("hi there pt 4")

        return (w1,h34),(w2,h34)

    def lane_warp(self, img):
        # print("hi there pt 2")
        extending_width = 463 #estimated from trial and error

        extension = extending_width // 2 # Calculate the amount of extension required on both sides
        extended_img = np.zeros((img.shape[0], img.shape[1] + extending_width, img.shape[2]), dtype=img.dtype) # Create a new array with the required shape and fill it with zeros
        extended_img[:, extension:extension + img.shape[1],] = img # Copy the input image to the middle of the new array

        pt1 = (0, 480) #bottom left
        pt2 = (640 + extending_width, 480) #bottom right

        h_i = 247 #estimated from trial & error

        h34 = 308 #estimated from trial & error
        pt3, pt4 = self.point_generator(pt1, pt2, h_i, h34, extending_width)

        birds_eye_img = self.birds_eye_view(extended_img, pt1, pt3, pt4, pt2)
        
        resized_h = 480
        resized_w = int(round(480*100/140, 0)) #calculated using pixel to cm ratio, 140 cm = 480 pixels => 100 cm = ? pixels
        dim = (resized_w, resized_h) # resizing to IRL aspect ratio
        
        resized_img = cv2.resize(birds_eye_img, dim, interpolation = cv2.INTER_AREA)
        # print("hi there pt 7")
        
        return resized_img

    def track_points_direction(self, img_file): #assuming img_file is the image array after reading
        # print("hi there")
        # cv2.imshow("Original Image", img_file)
        # cv2.waitKey(1)

        warped_img = self.lane_warp(img_file)
        # cv2.imshow("Lane warped Image", warped_img)
        # cv2.waitKey(1)

        # print("hi there pt 8")

        # Convert the image to grayscale
        gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("Grayscale", gray)
        # cv2.waitKey(1)

        # Define the kernel size and standard deviation for the Gaussian Blur
        kernel_size = (5, 5) #or(7, 7) might also be good
        sigma = 1.0 #0 does nothing, and 1 is in general good but we can adjust this with trial and error

        # Apply the Gaussian Blur
        blurred = cv2.GaussianBlur(gray, kernel_size, sigma)
        # cv2.imshow("Blurred", blurred)
        # cv2.waitKey(1)

        # apply thresholding to convert grayscale to binary image
        # ret,thresh = cv2.threshold(blurred,210,255,0)
        # adaptive_threshold = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 81, -40) #for mean c adaptive thresholding
        adaptive_threshold = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 231, -60)

        # Display the Binary Image after adaptive thresholding
        # cv2.imshow("Adaptive Binary Image", adaptive_threshold)
        # cv2.waitKey(1)  

        contours =[]
        #now we will find the centroids of the lane lines
        # Find the contours of the white blobs
        contours, _ = cv2.findContours(adaptive_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # approximate lane width, estimated
        min_width = 30
        max_width = 90
    
        # approximate lane height, estimated
        min_height = 80
        max_height = 180

        # approximate lane area
        min_area = round(min_width * min_height)
        max_area = round(max_width * max_height)

        centroids = []

        for contour in contours:

            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True) #get approx number of sides of the contour
            lane_area = cv2.contourArea(contour)

            x,y,w,h = cv2.boundingRect(contour)
            if w < min_width or w > max_width or h < min_height or h > max_height:
                continue  # ignore rectangles that aren't the expected size
        
            if (lane_area > min_area) and (lane_area < max_area) and (len(approx) == 4): #when the contour might be the lane

                # Compute the moments of the contour
                moments = cv2.moments(contour)
            
                # Compute the center pixel location of the lane
                if moments["m00"] != 0:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                    center = (x, y)
                    centroids.append(center)
                
                    # Draw a circle at the center pixel location
                    cv2.circle(warped_img, center, 5, (255, 0, 0), -1)
                    '''
                    error_x = x - 343/2
                    speed_cmd = Twist()
                    speed_cmd.linear.x = 1 
                    speed_cmd.angular.z = -error_x / 100
                    self.speed_pub.publish(speed_cmd)
                    '''
                continue

        #centroids_list = [list(p) for p in centroids]
        
        return warped_img, centroids #this will now contain the center points of the lanes

    def get_point_to_follow(self, img_centroids):
        num_detected_lanes = len(img_centroids)
        husarion_pos = (172, 597) # estimated position of husarion using aspect ratio

        if num_detected_lanes == 1: #go towards that point
            return img_centroids[0]

        elif num_detected_lanes > 1: #go towards the closest point
            min_h = img_centroids[0][1]
            pt_w = img_centroids[0][0]
            for pt in img_centroids:
                if min_h < pt[1]:
                    min_h = pt[1]
                    pt_w = pt[0]
            return (pt_w,min_h)
    
        #the else condition will never happen since img_centroids will always return at least 1 pt


def main():
    Showing_image_object = LaneDetector()
    # Showing_image_object.run()
    #cv2.imshow("hi",img[-1])
    #cv2.waitKey(1)
    
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting DOwn")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()