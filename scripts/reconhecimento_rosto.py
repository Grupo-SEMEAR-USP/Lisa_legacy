#!/usr/bin/python3

# Import the necessary libraries
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

def find_face(frame):
    # Convert frame to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the image and detect faces
    results = face_detection.process(rgb_frame)
    
    if results.detections:
        # Initialize variables to store the largest face
        max_area = 0
        largest_face = None
        
        for detection in results.detections:
            # Get the bounding box of the detected face
            bboxC = detection.location_data.relative_bounding_box
            ih, iw, _ = frame.shape
            bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                   int(bboxC.width * iw), int(bboxC.height * ih)
            
            # Calculate the area of the bounding box
            area = bbox[2] * bbox[3]
            
            # Update largest face if this one is bigger
            if area > max_area:
                max_area = area
                largest_face = frame[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
        
        # Return the largest face region
        return largest_face
    
    # Return None if no faces are detected
    return None

def callback(data):
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data)
    face_frame = find_face(current_frame)
    # rospy.loginfo('Encontrando Rostos')
    
    if face_frame is not None:
        # Convert the face frame back to a ROS Image message
        face_msg = br.cv2_to_imgmsg(face_frame, encoding="passthrough")
        pub.publish(face_msg)
        face_detected_pub.publish(True)
    else:
        face_detected_pub.publish(False)

def publish_faces():
    global pub, face_detected_pub
    pub = rospy.Publisher('/Rostos', Image, queue_size=10)
    face_detected_pub = rospy.Publisher('/face_detected', Bool, queue_size=10)
    rospy.init_node('encontrar_rostos_pub_py', anonymous=True)
    rospy.Subscriber('/Imagens', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_faces()
    except rospy.ROSInterruptException:
        pass
