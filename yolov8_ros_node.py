import cv2
import time
import math
import rclpy
import torch
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Define the minimum confidence threshold for a person detection
PERSON_CONFIDENCE_THRESHOLD = 0.70 # 80%

class YoloPoseCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_camera_node')
        self.get_logger().info('YOLO-Pose ROS Node Starting...')

        # --- 1. Model Loading (Omitted for brevity, assumed success) ---
        self.model = None # Initialize attribute to None
        # ... (Your existing model loading code here) ...
        try:
            self.model = YOLO('yolov8n-pose.pt') 
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model.to(self.device)
            self.get_logger().info(f'YOLO-Pose Model loaded successfully on device: {self.device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO-Pose model: {e}')
            
        # --- 2. ROS 2 Setup ---
        self.br = CvBridge()
        self.bridge = CvBridge() #Profundidad
        #----------------------------------------------------------------------------------------------------
        #Suscritor Camara de profundidad
        self.subscription_Camara_Profundidad = self.create_subscription(
            Image,
            '/a200_0000/sensors/camera_0/depth/image',  #Topico de la camara de profundidad
            self.listener_Camara_Profundidad,
            10)
        #----------------------------------------------------------------------------------------------------
        #Suscritor de la imagen a color para la deteccion de personas
        self.subscription_Camara_RGB = self.create_subscription(
            Image,
            '/a200_0000/sensors/camera_0/color/image',
            self.Deteccion_Camara_RGB,
            10
        )
        
        # --- Publicador de estado de navegacion ---
        self.Navegador_publicador = self.create_publisher(
            Bool, 
            '/Activacion_navegacion', 
            10)
        
        #Creamos el publicador para cmd_vel que dirija el robot cuando se detecte una persona
        self.Robot_Clearpath_cmd_vel = self.create_publisher(
            Twist,
            '/a200_0000/cmd_vel',
            10)
        
        timer_period = 0.5
        self.detect_personas = False
        self.cx = 0
        self.cy = 0
        self.Error_x_persona = 0
        self.distancia_Persona = 0
        self.subscription_Camara_RGB
        self.subscription_Camara_Profundidad

    def Deteccion_Camara_RGB(self, data):
        if self.model is None:
            self.get_logger().warn('Model is not initialized. Skipping frame.', throttle_duration_sec=5.0)
            return

        try:
            cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
            
        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn('Received empty image frame.')
            return
            
        rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        # --- 4. Run Inference ---
        results = self.model(rotated_image, verbose=False) 

        # --- 5. Process and Publish Results ---
        
        # Array to hold all the pose data for publishing
        pose_data_array = []
        
        person_detected_frame = False
        
        if results and len(results) > 0:
            result = results[0]
            if result.keypoints is not None and result.boxes is not None:
                
                for box, kps_tensor in zip(result.boxes, result.keypoints.data):
                    
                    person_conf = box.conf.item()
                    
                    if person_conf >= PERSON_CONFIDENCE_THRESHOLD:
                        
                        person_data = [person_conf]
                        kps_flat = kps_tensor.cpu().numpy().flatten().tolist()
                        person_data.extend(kps_flat)
                        pose_data_array.extend(person_data)
                            
                        # 1. Centro del Bounding Box
                        bbox = box.xyxy.cpu().numpy()[0]
                        self.cx = int((bbox[0] + bbox[2]) / 2)
                        self.cy = int((bbox[1] + bbox[3]) / 2)

                        keypoints_xy = kps_tensor[:, :2].cpu().numpy()
                            
                        # Índices: 5 (Hombro Izq), 6 (Hombro Der), 11 (Cadera Izq), 12 (Cadera Der)
                        if len(keypoints_xy) >= 13:
                            shoulder_l = keypoints_xy[5]
                            shoulder_r = keypoints_xy[6]
                            hip_l = keypoints_xy[11]
                            hip_r = keypoints_xy[12]

                            torso_points = np.array([shoulder_l, shoulder_r, hip_l, hip_r])
                                
                            # Filtrar puntos que sean (0,0) (no detectados)
                            valid_points = torso_points[np.sum(torso_points, axis=1) > 0]

                            if len(valid_points) > 0:
                                self.torso_cx = int(np.mean(valid_points[:, 0]))
                                self.torso_cy = int(np.mean(valid_points[:, 1]))
                                person_detected_frame = True
        
        self.detect_personas = person_detected_frame		#Guarda si persona esta detectada
        
        if person_detected_frame:
            cv2.circle(rotated_image, (self.cx, self.cy), 8, (0, 255, 0), -1) # Verde: BBox
            cv2.circle(rotated_image, (self.torso_cx, self.torso_cy), 8, (255, 0, 0), -1) # Azul: Torso
            cv2.putText(rotated_image, "Target", (self.torso_cx+10, self.torso_cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        
        rendered_frame = results[0].plot()
        cv2.imshow('YOLO-Pose Detector de personas', rendered_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quitting YOLO-Pose Display...')
            rclpy.shutdown() 

    def listener_Camara_Profundidad(self, msg):
        try:
            # Convertir la imagen de ROS a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  # Tipo de profundidad (32 bits de flotante, un solo canal)
            
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            
            msg_vel = Twist()
            Navegar_booleano = Bool()
            
            height, width = cv_image.shape
            self.center_x = width // 2
            
            if self.detect_personas == True and self.torso_cx < width and self.torso_cx > 0 and self.torso_cy < height and self.torso_cy > 0:
                self.distancia_Persona = cv_image[self.torso_cy, self.torso_cx]
                self.get_logger().info(f'Distancia de la persona detectada: {self.distancia_Persona} metros')
                
                Navegar_booleano.data = False
                
                self.Navegador_publicador.publish(Navegar_booleano)
                
                self.Error_x_persona = self.center_x - self.torso_cx
                
                Vmax = 1.0
                Kw = 0.005
                
                v = Vmax*np.tanh(self.distancia_Persona)
                w = Kw*self.Error_x_persona
            
                msg_vel.angular.z = w
                if (self.distancia_Persona <= 2.5):
                    msg_vel.linear.x = 0.0
                else:
                    msg_vel.linear.x = v
                
                """if (self.Error_x_persona == 0.0):
                    msg_vel.linear.x = v
                    if (self.distancia_Persona <= 2.0):
                        msg_vel.linear.x = 0.0
                else:
                    msg_vel.angular.z = w
                    msg_vel.linear.x = 0.0"""
        
                self.Robot_Clearpath_cmd_vel.publish(msg_vel)
                
            elif self.detect_personas == False:
                self.get_logger().info('no se ha detectado persona')
                self.Error_x_persona = 0
        except Exception as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    except Exception as e:
        if rclpy.ok():
            node.get_logger().error(f'An error occurred during spin: {e}')
    finally:
        if rclpy.ok():
            node.get_logger().info('Cleaning up resources...')
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
