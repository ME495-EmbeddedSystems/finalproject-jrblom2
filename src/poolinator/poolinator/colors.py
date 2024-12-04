import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30, record_filename=None, play_back=None):
        # initialize pipeline and configure streams
        # pipeline handles data streams (depth and color)
        # config is used to set up parameters (resolution, format, frame rate) of the strems 
        # enable stream specifies width, height and frame rate
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # both lines from below are moved into if else statement if playback
        #self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        #self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # if there is playback play it back if not just do the live feed like before
        if play_back:
            #rs.pipeline.start(config)
            print('IN PLAYBACK MODE')
            self.config.enable_device_from_file(play_back)
           
        else:
            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

            if record_filename:
                print('RECORDING')
                self.config.enable_record_to_file(record_filename)
                


        # starts pipeline --> starts camera 
        self.profile = self.pipeline.start(self.config)

        # we now only need to do depth sensor stuff if not playing back
        # deal with depth sensors scale --> go from pixels to meters
        if not play_back:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"Depth Scale is: {self.depth_scale}")

            # clipping distance is the max distance the camera can capture
            # everyhting furhter away than 1 meter will be gray
            # define clipping distance in meters  
            self.clipping_distance_in_meters = 1
            self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        else: 
            #was gettign an error bc play back doesnt need depth scale or clipping distance
            #so lets set each to zero when in playback mode
            self.depth_scale = 1 
            self.clipping_distance = 1


        # create align object -> this means something like matching depth data to color data
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def get_aligned_frames(self):
        try:
            # wait for frames and align them 
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            # get aligned depth and color frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            #if not alligned dont return anything
            if not aligned_depth_frame or not color_frame:
                return None, None

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return depth_image, color_image
        except Exception as e:
            print(f"Error while getting aligned frames: {e}")
            return None, None

    def process_frames(self, depth_image, color_image):
        # Remove background - Set pixels further than clipping_distance to grey (153)
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Generate depth colormap
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        return images
    
    
    def color_tracking(self, color_image):
        if color_image is None:
            return None, None

        #convert BGR TO HSV
        hsv = cv2.cvtColor (color_image, cv2.COLOR_BGR2HSV )

        ########## Begin Citation ##########
        # Lower red range (0 to 10)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        
        # Upper red range (170 to 180)
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Threshold to get only the red regions
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        ######### End Citation ############

        # Combine both masks
        red_mask = cv2.bitwise_or(mask1, mask2)

        # isolate green surface
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Remove green areas from the red mask to isolate only the red ball
        red_ball_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(green_mask))

        # Apply the mask to the original image to extract the red ball
        red_ball = cv2.bitwise_and(color_image, color_image, mask=red_ball_mask)

        # Check if any red pixels detected
        if np.sum(red_ball_mask) == 0:
            print("No red ball detected!")
            return None, None

        return red_ball_mask, red_ball


    def stop(self):
        self.pipeline.stop()



    def display_frames(self, images, mask= None, res= None):

        # Render the main aligned images
        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        if images is not None:
            cv2.imshow('Align Example', images)

        if mask is not None:
            cv2.namedWindow('Red Ball Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Red Ball Mask', mask)

        if res is not None:
            cv2.namedWindow('Red Ball Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Red Ball Detection', res)

        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            return True
        return False
    
        
        
        
# creates an instance fo RealSenseCamera class
# in the loop frames are captured prcoessed and displayeed 
if __name__ == "__main__":
    camera = RealSenseCamera()

    try:
        counter = 1
        while True:
            counter += 1
            print(f'Processing frame {counter}')

            depth_image, color_image = camera.get_aligned_frames()

            if depth_image is None or color_image is None:
                print(f"Frame {counter} not valid")
                continue

            images = camera.process_frames(depth_image, color_image)

            # Track the red ball
            red_mask, red_ball = camera.color_tracking(color_image)

            # Display the frames
            if camera.display_frames(images, mask=red_mask, res=red_ball):
                break
    finally:
        camera.stop()

