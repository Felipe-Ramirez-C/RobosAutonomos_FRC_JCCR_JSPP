import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from statistics import harmonic_mean
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/a200_0000/sensors/lidar2d_0/scan',
            self.scan_callback,
            10)
            
        self.Robot_Pose_Subscriptor = self.create_subscription(
            Odometry,
            '/a200_0000/platform/odom/filtered',
            self.Robot_Odometria,
            10)
            
            
        self.Robot_IMU_Subscriptor = self.create_subscription(
            Imu,
            '/a200_0000/sensors/imu_0/data',
            self.Robot_IMU_Callback,
            10)
            
        self.Navegador_Act_suscriptor = self.create_subscription(
            Bool,
            '/Activacion_navegacion',
            self.navegacion_Act_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
         
        self.yaw_deg_IMU = 0.0
        self.yaw_deg = 0.0
        self.Pared_Der_Dect = False
        self.estado = 0
        self.ang_offset_robot_giro = 0.0
        self.seguro_inicial_ang = False
        self.Robot_linear_Vel = 0.0
        self.Robot_angular_Vel = 0.0
        self.Robot_is_stopped = False
        self.segundo_giro = False
        self.tercer_giro = False
        self.stop_start_time = None
        self.navegar = True
        
        self.Robot_Pose_Subscriptor
        self.Robot_IMU_Subscriptor
        self.Navegador_Act_suscriptor
        self.subscription
        
        
    def Robot_Odometria(self, Odom_msg):
        self.x_robot = Odom_msg.pose.pose.position.x
        self.y_robot = Odom_msg.pose.pose.position.y
        
        self.Robot_linear_Vel = Odom_msg.twist.twist.linear.x
        self.Robot_angular_Vel = Odom_msg.twist.twist.angular.z
        
        ###################################################################################
        #Convertimos el valor del angulo en quarteniones del robot en valores de angulos de euler
        qx = Odom_msg.pose.pose.orientation.x
        qy = Odom_msg.pose.pose.orientation.y
        qz = Odom_msg.pose.pose.orientation.z
        qw = Odom_msg.pose.pose.orientation.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.yaw_deg = np.degrees(yaw)
        #self.yaw_deg = self.yaw_deg if self.yaw_deg >= 0 else (360 + self.yaw_deg)
        
        #self.get_logger().info(f"Angulo Robot yaw: {self.yaw_deg:.2f}")
        
    def Robot_IMU_Callback(self, IMU_msg):
        qx_IMU = IMU_msg.orientation.x
        qy_IMU = IMU_msg.orientation.y
        qz_IMU = IMU_msg.orientation.z
        qw_IMU = IMU_msg.orientation.w
        
        siny_cosp_IMU = 2 * (qw_IMU * qz_IMU + qx_IMU * qy_IMU)
        cosy_cosp_IMU = 1 - 2 * (qy_IMU * qy_IMU + qz_IMU * qz_IMU)
        yaw_IMU = np.arctan2(siny_cosp_IMU, cosy_cosp_IMU)
        self.yaw_deg_IMU = np.degrees(yaw_IMU)
        self.seguro_inicial_ang = True
        #self.yaw_deg_IMU = self.yaw_deg_IMU if self.yaw_deg_IMU >= 0 else (360 + self.yaw_deg_IMU)
        
        #self.get_logger().info(f"Angulo Robot yaw IMU: {self.yaw_deg_IMU:.2f}")
        
        
    def navegacion_Act_callback(self, msg_act):
        self.navegar = msg_act.data
        self.get_logger().info(f"Persona encontrada, Deteniendo navegacion")

    def scan_callback(self, msg):
        msg_vel = Twist()
        
        Vmax = 1.0		#Valores del controlador 1 del robot
        Kw = 0.02
        
        #NOTA: El sensor laser tiene un rango de vision de entre 135° y -135°, por lo que tiene 270° de angulo de vision, con un total de 540 puntos de referencia del sensor lidar (0, 539) = (-135, 135), con un paso de de 0.5° entre cada punto del laser
        frontal_centro_max = 270	# Equivalente a 0° del robot
        frontal_der_max = 180		# Equivalente a -45° del robot
        frontal_izq_max = 360		# Equivalente a 45° del robot
        
        Prueba_lateral_22 = 135		# Equivalente a - 22.5° del robot
        
        Derecho_sup_lim = 90		# Equivalente a -90°
        Derecho_inf_lim = 0		# Equivalente a -135°
        
        Izquierda_sup_lim = 450		# Equivalente a 90°
        Izquierda_inf_lim = 540		# Equivalente a 135°

        Frontal_Der_dist = self.get_prom_range(msg.ranges, frontal_der_max, frontal_centro_max)
        Frontal_Izq_dist = self.get_prom_range(msg.ranges, frontal_centro_max, frontal_izq_max)
        Derecha_sup = self.get_prom_range(msg.ranges, Derecho_sup_lim, frontal_der_max)
        Derecha_inf = self.get_prom_range(msg.ranges, Derecho_inf_lim, Derecho_sup_lim)
        
        Frontal_Central = self.get_prom_range(msg.ranges, frontal_centro_max - 60, frontal_centro_max + 3)		#Frente directo del Robot
        Derecha_Central = self.get_prom_range(msg.ranges, Derecho_sup_lim - 10, Derecho_sup_lim + 10)			#Lateral derecho directo del Robot
        
        Der_alineacion = self.get_prom_range(msg.ranges, Derecho_sup_lim - 2, Derecho_sup_lim + 2)
        Lateral_alineacion = self.get_prom_range(msg.ranges, frontal_der_max - 2, frontal_der_max + 20)			#Alineacion de -45°
        
        Frontal_media_test = self.get_prom_range(msg.ranges, frontal_der_max, frontal_izq_max)
        
        # Debug: Imprimir distancias para calibrar
        #self.get_logger().info(f'IMU: {self.yaw_deg_IMU:.2f}, Odom: {self.yaw_deg:.2f}')
        
        #self.get_logger().info(f"Rango Frente-Der: {Frontal_Der_dist:.2f}")
        #self.get_logger().info(f"Rango Frente-Izq: {Frontal_Izq_dist:.2f}")
        #self.get_logger().info(f"Rango Der-superior: {Derecha_sup:.2f}")
        #self.get_logger().info(f"Rango Der-inferior: {Derecha_inf:.2f}")
        
        #Pruebas del controlador para wall tracking
        #########################################################################
        
        if (self.navegar == True):
            #Fase 0 Toma el angulo del robot antes del movimiento inicial
            if (self.estado == 0 and self.seguro_inicial_ang == True):
                self.get_logger().info(f'Fase 0 Toma el angulo del robot para ajustar el giro como necesito')
                self.ang_offset_robot_giro = self.yaw_deg_IMU
                self.get_logger().info(f'Test: {self.ang_offset_robot_giro:.2f}')
                self.estado = 1
        
            #Primera Fase Ubicacion de pared 1
            if (Derecha_sup <= 9.5 and self.estado == 1 and self.Pared_Der_Dect == False):
                self.get_logger().info(f'Fase 1 Buscando pared 1')
                target_yaw_giro = self.ang_offset_robot_giro - 45
                target_yaw_giro = (target_yaw_giro + 180) % 360 - 180	#Normalizamos el dato para que este en el espetro de datos que se utilizan para el robot
                Error_yaw = (target_yaw_giro) - self.yaw_deg_IMU		#Error de angulo para el controlador de navegador en la primera estancia
        
                v = Vmax*np.tanh((Frontal_Central - 0.5))
                w = Kw*Error_yaw
        
                self.get_logger().info(f'IMU: {self.yaw_deg_IMU:.2f}, Distancia_Frontal: {Frontal_Central:.2f}, Error: {Error_yaw:.2f}')
        
                if (Error_yaw <= 0.2 and Error_yaw >= -0.2):
                    msg_vel.linear.x = v
                    msg_vel.angular.z = 0.0
                    if (Frontal_Central <= 0.6):
                        msg_vel.linear.x = 0.0
                        msg_vel.angular.z = 0.0
                        self.ang_offset_robot_giro = self.yaw_deg_IMU
                        self.estado = 2
                else:
                    msg_vel.linear.x = 0.0
                    msg_vel.angular.z = w
            #Segunda Fase Redireccionar al robot y verificar la distancia con el sensor del lado derecho
            if (self.estado == 2):
                self.get_logger().info(f'Fase 2 Transicion y enderezando el robot')
                target_yaw_giro = self.ang_offset_robot_giro + 45		#Endera al robot para estar alineado con la pared
                target_yaw_giro = (target_yaw_giro + 180) % 360 - 180
                Error_yaw = target_yaw_giro - self.yaw_deg_IMU
                w = Kw*Error_yaw
            
                self.get_logger().info(f'IMU: {self.yaw_deg_IMU:.2f}, Error: {Error_yaw:.2f}')
            
                if (Error_yaw <= 0.2 and Error_yaw >= -0.2):
                    msg_vel.angular.z = 0.0
                    self.ang_offset_robot_giro = self.yaw_deg_IMU
                    self.estado = 3
                else:
                    msg_vel.angular.z = w
            #Fase 3 Identificacion de pared y habilitacion del seguimiento
            if (self.estado == 3):
                self.get_logger().info(f'Fase 3 Controlador wall-tracking')
                self.get_logger().info(f'Der_Sup: {Derecha_sup:.2f}, Der_Central: {Derecha_Central:.2f}, Der_inf: {Derecha_inf:.2f}')
                
                Dist_proy = Lateral_alineacion * np.cos(45)
                Error_Alineacion = Der_alineacion - Dist_proy		#Calculo del error de alineacion
                
                v = Vmax*np.tanh((Frontal_Central - 0.5))
            
                Error_pared = 0.6 - Der_alineacion
            
                Kp_dist = 2.2
                Kp_alineacion = 1.6
            
                correcion_giro = (Error_pared*Kp_dist) + (Error_Alineacion*Kp_alineacion)
            
                self.get_logger().info(f'Err Alineación: {Error_Alineacion:.2f}, Distancia: {Der_alineacion:.2f}, Distancia Frontal: {Frontal_Central:.2f}')
            
                msg_vel.linear.x = v
                msg_vel.angular.z = max(min(correcion_giro, 1.0), -1.0)
            
            
                self.Robot_is_stopped = abs(self.Robot_linear_Vel) < 0.002 and abs(self.Robot_angular_Vel) < 0.002
                #print(self.Robot_is_stopped)
                if self.Robot_is_stopped:
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now()
                        self.get_logger().info("Robot Detenido, comenzando conteo")
                        self.ang_offset_robot_giro = self.yaw_deg_IMU
                    else:
                        current_time = self.get_clock().now()
                    
                        time_elapsed = current_time - self.stop_start_time
                    
                        segundos_parado = time_elapsed.nanoseconds/1e9
                    
                        if segundos_parado > 5.0:
                            self.get_logger().info("Robot bloqueado, iniciando funcion de rescate")
                            #Pasamos de estado a estado de rescate, que va a hacer un recorrido ideal para la habitacion con mesas en forma de L
                            self.get_logger().info(f'angulo actual robot: {self.ang_offset_robot_giro:.2f}')
                            self.estado = 4
                else:
                    if self.stop_start_time is not None:
                        self.get_logger().info("Robot moviendose, Reiniciando el cronometro")
                        self.stop_start_time = None
            
            #Fase 4 Seguimiento y minicontrolador de distancia con la pared
            if (self.estado == 4):
                self.get_logger().info(f'Fase 4 Funcion de rescate para habitacion con mesas en forma de L')
            
                #Primer giro y reversa
                if (self.Pared_Der_Dect == False):
                    v = Vmax*np.tanh((Frontal_Central - 0.9))
            
                    #self.get_logger().info(f'V: {v:.2f}, angulo: {self.ang_offset_robot_giro:.2f}')
            
                    if (Frontal_Central >= 0.8):
                        msg_vel.linear.x = 0.0
                        target_yaw_giro = self.ang_offset_robot_giro + 90
                        target_yaw_giro = (target_yaw_giro + 180) % 360 - 180
                
                        Error_yaw = target_yaw_giro - self.yaw_deg_IMU
                        w = Kw*Error_yaw
                
                        if (Error_yaw <= 0.2 and Error_yaw >= -0.2):
                            msg_vel.angular.z = 0.0
                            self.ang_offset_robot_giro = self.yaw_deg_IMU
                            self.Pared_Der_Dect = True
                        else:
                            msg_vel.angular.z = w
                    else:    
                        msg_vel.linear.x = v
                #Segundo giro
                if (self.Pared_Der_Dect == True and self.segundo_giro == False):
                    v = Vmax*np.tanh((Frontal_Central - 0.5))
                
                    if (Frontal_Central <= 0.6):
                        msg_vel.linear.x = 0.0
                        self.segundo_giro = True
                    else:    
                        msg_vel.linear.x = v
                    
                if (self.segundo_giro == True):        
                    target_yaw_giro = self.ang_offset_robot_giro + 90
                    target_yaw_giro = (target_yaw_giro + 180) % 360 - 180
                    
                    self.get_logger().info(f'Frontal: {Frontal_Central:.2f}, angulo target: {target_yaw_giro:.2f}, angulo del robot: {self.yaw_deg_IMU:.2f}')
                
                    Error_yaw = target_yaw_giro - self.yaw_deg_IMU
                    w = Kw*Error_yaw
                
                    if (Error_yaw <= 0.2 and Error_yaw >= -0.2):
                        msg_vel.angular.z = 0.0
                    
                        v = Vmax*np.tanh((Frontal_Central - 0.7))
                    
                        if (Frontal_Central <= 0.75):
                            msg_vel.linear.x = 0.0
                            self.ang_offset_robot_giro = self.yaw_deg_IMU
                            self.tercer_giro = True
                        else:    
                            msg_vel.linear.x = v    
                    else:
                        msg_vel.angular.z = w
                #Tercer giro
                if (self.tercer_giro == True):
                    target_yaw_giro = self.ang_offset_robot_giro + 160
                    target_yaw_giro = (target_yaw_giro + 180) % 360 - 180
                
                    self.get_logger().info(f'Frontal: {Frontal_Central:.2f}, angulo target: {target_yaw_giro:.2f}, angulo del robot: {self.yaw_deg_IMU:.2f}')
                
                    Error_yaw = target_yaw_giro - self.yaw_deg_IMU
                    w = Kw*Error_yaw
                
                    if (Error_yaw <= 0.2 and Error_yaw >= -0.2):
                        msg_vel.angular.z = 0.0
                        self.Pared_Der_Dect = False
                        self.segundo_giro = False
                        self.tercer_giro = False
                        self.stop_start_time = None
                        self.estado = 3
                    else:
                        msg_vel.angular.z = w
        
            self.publisher.publish(msg_vel)

    def get_prom_range(self, ranges, start_idx, end_idx):
        # Función auxiliar para limpiar datos (ignorar infs y nans)
        slice_ranges = ranges[start_idx:end_idx]
        valid_ranges = [r for r in slice_ranges if r > 0.05 and r < 10.0]
        if not valid_ranges:
            return 10.0 # Si no ve nada, asumir espacio libre
        #return harmonic_mean(valid_ranges)
        return min(valid_ranges)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
