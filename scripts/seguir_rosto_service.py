# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import mediapipe as mp
# from mediapipe.tasks import python
# from mediapipe.tasks.python import vision
# from std_srvs.srv import Trigger, TriggerResponse
# import os
# import rospkg

# rospack = rospkg.RosPack()
# package_path = rospack.get_path('estrutura')
# model_path = os.path.join(package_path, 'Models', 'gesture_recognizer.task')

# class FaceDirectionService:
#     def __init__(self):
#         rospy.init_node('face_direction_service_node')
#         self.bridge = CvBridge()
#         self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.2)
#         self.service = rospy.Service('seguir_rostos_service', SeguirRostos, self.handle_follow)

#     def handle_follow(self, req):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(req.image, "passthrough")
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")
#             return SeguirRostosResponse(success=False, message="CvBridge Error")

#         # Converte a imagem para RGB
#         image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         # Processa a imagem para detectar rostos
#         results = self.face_detection.process(image_rgb)

#         if results.detections:
#             for detection in results.detections:
#                 bbox = detection.location_data.relative_bounding_box
#                 height, width, _ = frame.shape
#                 x_center = int(bbox.xmin * width + (bbox.width * width) / 2)
#                 y_center = int(bbox.ymin * height + (bbox.height * height) / 2)

#                 # Calcula as distâncias
#                 rect_center_x = width // 2
#                 rect_center_y = height // 2
#                 distance_x = x_center - rect_center_x
#                 distance_y = y_center - rect_center_y

#                 return SeguirRostosResponse(success=True, message="Rosto detectado", center_x=distance_x, center_y=distance_y)

#         return SeguirRostosResponse(success=False, message="Nenhum rosto detectado")

# def main():
#     FaceDirectionService()
#     rospy.spin()

# if __name__ == '__main__':
#     main()
