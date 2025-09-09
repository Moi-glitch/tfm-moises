#!/usr/bin/env python3


import time
import math

import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Parámetros de la cámara
IMAGE_WIDTH = 320           # px (actualizar según la resolución de la cámara)
CENTER_X = IMAGE_WIDTH / 2  # px
FOV_H_DEG = 120.0           # grados (campo de visión horizontal)
FOV_H_RAD = math.radians(FOV_H_DEG)


class DetectObject(Node):

    def __init__(self):
        super().__init__('detect_object')

        parameter_descriptor_hue = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],
            description='Hue Value (0~179)'
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            description='Saturation/Lightness Value (0~255)'
        )

        self.declare_parameter(
            'red.hue_l', 0, parameter_descriptor_hue)
        self.declare_parameter(
            'red.hue_h', 179, parameter_descriptor_hue)
        self.declare_parameter(
            'red.saturation_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.saturation_h', 255, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.lightness_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.lightness_h', 255, parameter_descriptor_saturation_lightness)

        self.declare_parameter('is_calibration_mode', False)

        self.hue_red_l = self.get_parameter(
            'red.hue_l').get_parameter_value().integer_value
        self.hue_red_h = self.get_parameter(
            'red.hue_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter(
            'red.saturation_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter(
            'red.saturation_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter(
            'red.lightness_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter(
            'red.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.get_detect_traffic_light_param)

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        self.counter = 1

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.get_image, 1)
        else:
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.get_image, 1)

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_light = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1)
        else:
            self.pub_image_traffic_light = self.create_publisher(
                Image, '/detect/image_output', 1)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_red_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1)
                self.pub_image_yellow_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1)
                self.pub_image_green_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub3/compressed', 1)
            else:
                self.pub_image_red_light = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1)
                self.pub_image_yellow_light = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1)
                self.pub_image_green_light = self.create_publisher(
                    Image, '/detect/image_output_sub3', 1)

        self.pub_color = self.create_publisher(String, '/color_detection', 10)

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_image_available = False
        self.is_traffic_light_finished = False

        self.status = 0
        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        self.off_traffic = False

        time.sleep(1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_detect_traffic_light_param(self, params):
        for param in params:
            if param.name == 'red.hue_l':
                self.hue_red_l = param.value
                self.get_logger().info(f'red.hue_l set to: {param.value}')
            elif param.name == 'red.hue_h':
                self.hue_red_h = param.value
                self.get_logger().info(f'red.hue_h set to: {param.value}')
            elif param.name == 'red.saturation_l':
                self.saturation_red_l = param.value
                self.get_logger().info(f'red.saturation_l set to: {param.value}')
            elif param.name == 'red.saturation_h':
                self.saturation_red_h = param.value
                self.get_logger().info(f'red.saturation_h set to: {param.value}')
            elif param.name == 'red.lightness_l':
                self.lightness_red_l = param.value
                self.get_logger().info(f'red.lightness_l set to: {param.value}')
            elif param.name == 'red.lightness_h':
                self.lightness_red_h = param.value
                self.get_logger().info(f'red.lightness_h set to: {param.value}')
        return SetParametersResult(successful=True)

    def get_image(self, image_msg):
        # Procesar cada frame
        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
                return

        self.is_image_available = True

    def timer_callback(self):
        if self.is_image_available:
            self.find_traffic_light()

    def find_traffic_light(self):
        self.detect_red_traffic_light()

        # Publicar la imagen con visualizaciones en el tópico /image_traffic_light
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, 'jpg'))
        else:
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))

    def pixel_to_angle(self, x_pixel: int) -> float:
        """
        Convierte la coordenada x del píxel (0…IMAGE_WIDTH-1) en un ángulo [rad]
        relativo al eje óptico (0 en el centro, positivo hacia la derecha).
        """
        dx = x_pixel - CENTER_X  # CENTER_X garantiza que el centro de la imagen sea 0 radianes
        theta = dx * (FOV_H_RAD / IMAGE_WIDTH)  # Escala dx según el campo de visión en radianes
        return theta  # Devuelve el ángulo en radianes

    def detect_red_traffic_light(self):
        # Convertir la imagen a HSV
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir el rango HSV para el rojo (en dos partes para manejar el ciclo del tono)
        lower_red_1 = np.array([0, self.saturation_red_l, self.lightness_red_l])  # Rango 1: 0–5
        upper_red_1 = np.array([5, self.saturation_red_h, self.lightness_red_h])

        lower_red_2 = np.array([175, self.saturation_red_l, self.lightness_red_l])  # Rango 2: 175–179
        upper_red_2 = np.array([179, self.saturation_red_h, self.lightness_red_h])

        # Crear máscaras para ambos rangos
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        # Combinar las máscaras
        mask = cv2.bitwise_or(mask1, mask2)

        # Aplicar un desenfoque gaussiano a la máscara
        mask = cv2.GaussianBlur(mask, (3, 3), 0)

        # Aplicar operaciones morfológicas
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Definir la ROI (región de interés)
        height, width = mask.shape[:2]
        roi_x_start = int(1/6 * width)
        roi_x_end = int(5/6 * width)
        roi_y_start = height // 3
        roi_y_end = height

        # Crear una máscara para la ROI
        roi_mask = np.zeros_like(mask)
        roi_mask[roi_y_start:roi_y_end, roi_x_start:roi_x_end] = 255

        # Combinar la máscara de la ROI con la máscara roja
        mask = cv2.bitwise_and(mask, roi_mask)

        # Invertir la máscara
        mask = cv2.bitwise_not(mask)

        # Detectar objetos rojos individuales
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10
        params.maxThreshold = 255
        params.filterByArea = True
        params.minArea = 700
        params.maxArea = 500000  # Incrementar el área máxima para permitir objetos grandes
        params.filterByCircularity = True  # Habilitar filtrado por circularidad
        params.minCircularity = 0.1       # Establecer circularidad mínima
        params.filterByConvexity = True   # Habilitar filtrado por convexidad
        params.minConvexity = 0.1         # Establecer convexidad mínima

        detector = cv2.SimpleBlobDetector_create(params)
        keypts = detector.detect(mask)

        # Crear una copia para visualización
        visualization_image = np.copy(self.cv_image)

        # Dibujar los límites de la ROI en la imagen de visualización
        cv2.rectangle(visualization_image, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 0), 2)

        # Lista para almacenar el centro de masa de cada objeto detectado
        detected_objects = []

        # Visualizar los objetos rojos detectados y calcular el centro de masa
        for i, keypt in enumerate(keypts):
            point_x = int(keypt.pt[0])
            point_y = int(keypt.pt[1])
            detected_objects.append({'id': i, 'x': point_x, 'y': point_y})

            # Calcular el ángulo del objeto detectado
            angle_rad = self.pixel_to_angle(point_x)
            angle_deg = math.degrees(angle_rad)  # Convertir radianes a grados

            # Dibujar un círculo verde en el centro de masa sobre la imagen de visualización
            cv2.circle(visualization_image, (point_x, point_y), 5, (0, 255, 0), -1)
            cv2.putText(visualization_image, f'ID={i}, At {angle_deg:.2f} Deg', (point_x + 10, point_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Publicar la información
            #timestamp = self.get_clock().now().to_msg().sec
            #self.pub_color.publish(String(data=f"red,angle={angle_rad:.2f},timestamp={timestamp}"))

            # recomendado para Gazebo según chatgpt
            now = self.get_clock().now().to_msg()
            timestamp = now.sec + now.nanosec * 1e-9
            self.pub_color.publish(String(data=f"red,angle={angle_rad:.2f},timestamp={timestamp:.6f}"))


            #self.get_logger().info(f'Objeto rojo detectado: ID={i}, angle={angle_deg:.2f}°, timestamp={timestamp}')


        # Mostrar la máscara combinada
        cv2.imshow("Combined Mask", mask)
        cv2.waitKey(1)

        # Publicar la imagen de visualización con marcadores en el tópico /image_traffic_light
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(visualization_image, 'jpg'))
        else:
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_imgmsg(visualization_image, 'bgr8'))

        # Publicar la máscara si se está en modo de calibración
        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
