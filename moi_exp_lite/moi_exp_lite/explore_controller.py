#!/usr/bin/env python3
import math
import os
from collections import deque
import yaml  # Importar el módulo YAML

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Importar configuraciones QoS
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan  # Para suscribirse a /scan
from visualization_msgs.msg import Marker  # Importar marcadores de RViz
from geometry_msgs.msg import PointStamped
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import OccupancyGrid

# Nodo que coordina la exploración y el marcado de objetos detectados
class ExploreController(Node):
    def __init__(self):
        super().__init__('explore_controller')


        # quitar si no va
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value


        # Declarar parámetros
        self.declare_parameter('name_object_thr', 0.05)
        self.declare_parameter('I_min', 200.0)
        self.declare_parameter('I_max', 6000.0)
        self.declare_parameter('bw_min', 0.01)
        self.declare_parameter('bw_max', 0.45)
        self.declare_parameter('cluster_radius', 10)

        # Ruta por defecto para guardar objetos detectados
        default_yaml = os.path.join(
            get_package_share_directory('moi_exp_lite'),
            'detected_objects.yaml')
        self.declare_parameter('yaml_file_path', default_yaml)

        # Obtener parámetros
        self.name_object_thr = self.get_parameter('name_object_thr').get_parameter_value().double_value
        self.I_min = self.get_parameter('I_min').get_parameter_value().double_value
        self.I_max = self.get_parameter('I_max').get_parameter_value().double_value
        self.bw_min = self.get_parameter('bw_min').get_parameter_value().double_value
        self.bw_max = self.get_parameter('bw_max').get_parameter_value().double_value
        self.cluster_radius = self.get_parameter('cluster_radius').get_parameter_value().integer_value
        self.yaml_file_path = self.get_parameter('yaml_file_path').get_parameter_value().string_value

        # Resolver ruta relativa al paquete si es necesario
        if not os.path.isabs(self.yaml_file_path):
            pkg_share = get_package_share_directory('moi_exp_lite')
            self.yaml_file_path = os.path.join(pkg_share, self.yaml_file_path)

        # Publicador para reanudar/detener la exploración
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # Publicador de marcadores para RViz
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Publicador que notifica cuando se marca un objeto rojo
        self.mark_pub = self.create_publisher(Bool, '/object_marked', 10)

        # Atributo para seguir los IDs de los marcadores
        self.marker_id = 0  # Inicializar ID de marcador

        # Suscribirse a detecciones de color
        self.color_sub = self.create_subscription(
            String,
            '/color_detection',
            self.object_detection,  # Método actualizado
            10
        )

        # Suscribirse al costmap para actualizaciones de la cuadrícula de ocupación
        self.costmap_sub = self.create_subscription(   
            OccupancyGrid,
            '/move_base/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.latest_costmap = None  # Inicializar atributo del costmap

        # Suscribirse a escaneos LiDAR con QoS de mejor esfuerzo y almacenar los últimos 20 mensajes
        self.scan_buffer = deque(maxlen=20)
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        # Temporizador para mantener la exploración en marcha si no se ha detenido aún
        self.stop_sent = False
        self.timer = self.create_timer(1.0, self.send_resume_if_not_stopped)

        # Lista para guardar datos de detección de color recibidos
        self.detected_data = []

        # Buffer y listener de TF para transformaciones de coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Lista para almacenar detecciones globales
        self.global_detections = []

    def get_robot_global_coordinates(self):
        """
        Obtener las coordenadas globales del robot en el marco del mapa.
        Devuelve una tupla (x_map, y_map) que representa la posición del robot en el mapa.
        """
        try:
            # Buscar la transformación del marco del mapa al marco base_link del robot
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            # Extraer la traslación (x, y) de la transformación
            x_tb3 = transform.transform.translation.x
            y_tb3 = transform.transform.translation.y

            #self.get_logger().info(f"Coordenadas globales del robot: x_map={x_tb3:.2f}, y_map={y_tb3:.2f}")
            return x_tb3, y_tb3

        except tf2_ros.LookupException:
            self.get_logger().error("TF lookup failed: Could not find transform from 'map' to 'base_link'.")
        except tf2_ros.ConnectivityException:
            self.get_logger().error("TF connectivity error: Could not connect to TF tree.")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("TF extrapolation error: Transform is not available for the requested time.")

        # Devolver None si la transformación falla
        return None, None

    def send_resume_if_not_stopped(self):
        """Publicar periódicamente un comando de reanudación a menos que ya se haya enviado uno de parada."""
        if not self.stop_sent:
            self.resume_pub.publish(Bool(data=True))


    def costmap_callback(self, msg: OccupancyGrid):
        """
        Callback para recibir el costmap más reciente y almacenarlo.
        Puede usarse para procesamientos o análisis posteriores.
        """
        self.latest_costmap = msg
        #self.get_logger().info(f"Costmap recibido con resolución {msg.info.resolution} m/pixel y tamaño {msg.info.width}x{msg.info.height} píxeles.")

    def costmap_to_binary_grid(self, costmap: OccupancyGrid, threshold=99):
        """Convertir el costmap en una cuadrícula 2D de 1s (ocupado) y 0s (libre)."""
        width = costmap.info.width
        height = costmap.info.height
        grid = [[0 for _ in range(width)] for _ in range(height)]
        for i, val in enumerate(costmap.data):
            if val >= threshold:
                x = i % width
                y = i // width
                grid[y][x] = 1
        return grid

    def world_to_grid(self, x, y, costmap: OccupancyGrid):
        """Convertir coordenadas del mundo a coordenadas de la cuadrícula."""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, costmap: OccupancyGrid):
        """Convertir coordenadas de la cuadrícula de nuevo a coordenadas del mundo."""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        x = gx * resolution + origin_x + resolution / 2.0
        y = gy * resolution + origin_y + resolution / 2.0
        return x, y

    def _extract_cluster(self, grid, gx, gy):
        """Rellenado sencillo para extraer un conjunto conectado de celdas ocupadas."""
        width = len(grid[0])
        height = len(grid)
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return []
        if grid[gy][gx] == 0:
            return []

        stack = [(gx, gy)]
        visited = set()
        cluster = []
        while stack:
            x, y = stack.pop()
            if (x, y) in visited:
                continue
            visited.add((x, y))
            if 0 <= x < width and 0 <= y < height and grid[y][x] == 1:
                cluster.append((x, y))
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    stack.append((x + dx, y + dy))
        return cluster

    def _cluster_center(self, cluster, costmap: OccupancyGrid):
        """Devolver las coordenadas del mundo para el centro de un clúster."""
        if not cluster:
            return None, None
        xs = [c[0] for c in cluster]
        ys = [c[1] for c in cluster]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        return self.grid_to_world(int(round(cx)), int(round(cy)), costmap)

    def get_obstacle_points(self,costmap:OccupancyGrid,threshold=99):
        """
        Extraer puntos de obstáculo del costmap basándose en un umbral.
        Devuelve una lista de coordenadas (x, y) de los obstáculos.
        """
        if costmap is None:
            self.get_logger().warn("No costmap available.")
            return []

        obstacle_points = []
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution

        for i in range(width * height):
            if costmap.data[i] >= threshold:
                # Convertir índice a coordenadas (x, y)
                x = (i % width) * resolution + costmap.info.origin.position.x
                y = (i // width) * resolution + costmap.info.origin.position.y
                obstacle_points.append((x, y))
        return obstacle_points


    def is_duplicate_detection(self, x_map, y_map, x_threshold=0.5, y_threshold=0.5):
        """
        Comprobar si las coordenadas dadas (x_map, y_map) ya están cerca de alguna detección global almacenada.
        Una detección se considera duplicada si cae dentro de una ventana de 1 metro (en x e y)
        respecto a cualquier coordenada almacenada.
        """
        self.get_logger().info(f"Checking duplicate detection for x_map={x_map}, y_map={y_map}")
        for detection in self.global_detections:
            stored_x = detection["x_map"]
            stored_y = detection["y_map"]

            # Comprobar si las nuevas coordenadas están dentro de la vecindad de las guardadas
            if abs(x_map - stored_x) <= x_threshold and abs(y_map - stored_y) <= y_threshold:
                self.get_logger().info(
                    f"Duplicate detection found: x_map={x_map:.2f}, y_map={y_map:.2f} is within the vicinity of stored detection x_map={stored_x:.2f}, y_map={stored_y:.2f}."
                )
                return True
        return False

    def save_detections_to_yaml(self):
        """
        Guardar las detecciones globales en un archivo YAML.
        """
        try:
            # Convertir objetos de NumPy a tipos estándar de Python
            detections_to_save = []
            for detection in self.global_detections:
                detections_to_save.append({
                    "color": detection["color"],
                    "timestamp": detection["timestamp"],
                    "x_map": float(detection["x_map"]),  # Convertir a float
                    "y_map": float(detection["y_map"])   # Convertir a float
                })

            with open(self.yaml_file_path, 'w') as yaml_file:
                yaml.dump(detections_to_save, yaml_file, default_flow_style=False)
            self.get_logger().info(f"Global detections saved to {self.yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save detections to YAML file: {e}")

    def marker_creation(self, final_angle, t_detect, distance):
        # Definir la distancia máxima para colocar el marcador
        max_distance = 2.0  # Límite de 2 metros

        # Comprobar si el objeto está dentro de la distancia permitida
        if distance > max_distance:
            self.get_logger().info(
                f"Object at distance {distance:.2f} m exceeds the maximum allowed distance of {max_distance} m. Marker not created."
            )
            return

        # Calcular (x, y) en el marco del robot usando el ángulo final
        x_robot = distance * math.cos(final_angle)
        y_robot = distance * math.sin(final_angle)

        self.get_logger().info(f"Computed object coordinates in robot frame: x={x_robot:.2f}, y={y_robot:.2f}")

        # Envolver el punto local en un PointStamped en el marco base_link
        pt_base = PointStamped()
        pt_base.header.stamp = self.get_clock().now().to_msg()
        pt_base.header.frame_id = "base_link"
        pt_base.point.x, pt_base.point.y, pt_base.point.z = x_robot, y_robot, 0.0

        try:
            self.get_logger().info(f"Attempting TF transformation for x_robot={x_robot}, y_robot={y_robot}")
            for _ in range(3):  # Reintentar hasta 3 veces
                try:
                    transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    break
                except tf2_ros.LookupException:
                    self.get_logger().warn("TF lookup failed, retrying...")
                    time.sleep(0.1)
            else:
                self.get_logger().error("TF lookup failed after retries.")
                return

            pt_map = do_transform_point(pt_base, transform)
            x_map, y_map = pt_map.point.x, pt_map.point.y
            self.get_logger().info(f"TF transformation successful: x_map={x_map}, y_map={y_map}")
        except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Ajustar las coordenadas usando el costmap, si es necesario
        obstacle_points = self.get_obstacle_points(self.latest_costmap)
        x_map, y_map = self.correction_via_costmap(x_map, y_map, obstacle_points)

        # Comprobar detección duplicada después de la corrección
        if self.is_duplicate_detection(x_map, y_map):
            self.get_logger().info(
                f"Duplicate detection discarded: x_map={x_map:.2f}, y_map={y_map:.2f}."
            )
            return

        # Crear y publicar el marcador para RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_object"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x_map
        marker.pose.position.y = y_map
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'🔴 Red detected at angle={final_angle * (180.0 / math.pi):.2f}°, timestamp={t_detect}. Marker placed at (x_map={x_map:.2f}, y_map={y_map:.2f}). Marker ID: {self.marker_id}.'
        )

        # Añadir la detección a la lista global de detecciones
        self.global_detections.append({
            "color": "red",
            "x_map": x_map,
            "y_map": y_map,
            "timestamp": self.get_clock().now().nanoseconds
        })

        # Guardar las detecciones actualizadas en el archivo YAML
        self.save_detections_to_yaml()

        # Notificar a otros componentes que se ha marcado un objeto rojo
        self.mark_pub.publish(Bool(data=True))

        # Incrementar el ID del marcador
        self.marker_id += 1

        # Limitar el tamaño de la lista de detecciones globales
        if len(self.global_detections) > 100:
            self.global_detections.pop(0)

    def correction_via_costmap(self, x_map, y_map, obstacle_points, radius=0.5):
        """
        Corregir las coordenadas (x_map, y_map) ajustándolas al obstáculo más cercano
        en el costmap o encontrando un objeto aislado a lo largo de la línea del robot al marcador.
        """
        if self.latest_costmap is None:
            self.get_logger().warn("No costmap available for correction.")
            return x_map, y_map

        if not obstacle_points:
            self.get_logger().warn("No obstacles found in the costmap.")
            return x_map, y_map

        costmap = self.latest_costmap
        grid = self.costmap_to_binary_grid(costmap)

        gx, gy = self.world_to_grid(x_map, y_map, costmap)
        width = len(grid[0])
        height = len(grid)

        # Función auxiliar para validar un clúster como objeto aislado
        def valid_cluster(cl):
            if not cl:
                return False
            xs = [c[0] for c in cl]
            ys = [c[1] for c in cl]
            w = max(xs) - min(xs) + 1
            h = max(ys) - min(ys) + 1
            area = len(cl)
            # Heurística: descartar clústeres muy grandes (probablemente paredes)
            return area < 400 and w < 20 and h < 20

        if 0 <= gx < width and 0 <= gy < height and grid[gy][gx] == 1:
            cluster = self._extract_cluster(grid, gx, gy)
            if valid_cluster(cluster):
                cx, cy = self._cluster_center(cluster, costmap)
                self.get_logger().info(f"Marker inside obstacle, snapping to object center ({cx}, {cy})")
                return cx, cy

        robot_x, robot_y = self.get_robot_global_coordinates()
        if robot_x is None or robot_y is None:
            self.get_logger().warn("Could not get robot global coordinates. Returning original coordinates.")
            return x_map, y_map

        steps = 100
        for step in range(1, steps + 1):
            t = step / steps
            wx = robot_x + t * (x_map - robot_x)
            wy = robot_y + t * (y_map - robot_y)
            gx, gy = self.world_to_grid(wx, wy, costmap)
            if 0 <= gx < width and 0 <= gy < height and grid[gy][gx] == 1:
                cluster = self._extract_cluster(grid, gx, gy)
                if valid_cluster(cluster):
                    cx, cy = self._cluster_center(cluster, costmap)
                    self.get_logger().info(f"Adjusted marker along line to ({cx}, {cy})")
                    return cx, cy

        self.get_logger().info("No suitable obstacle found; returning original coordinates.")
        return x_map, y_map

    def object_detection(self, msg: String):
        try:
            self.get_logger().info("Debug: Entering object_detection method")
            self.get_logger().info(f"Debug: Received message: {msg.data}")
            # Analizar el mensaje entrante
            data_parts = msg.data.split(',')
            color = data_parts[0].strip()
            angle_rad = float(data_parts[1].split('=')[1])
            t_detect = float(data_parts[2].split('=')[1])  # Usar marca de tiempo en flotante

            self.detected_data.append({
                'color': color,
                'angle': angle_rad,
                'timestamp': t_detect
            })

            # PRIMER CONTACTO CON EL OBJETO --------------------------------------------------
            if color.lower() == 'red' and not self.stop_sent:
                # Encontrar el escaneo cuyo header.stamp esté más cerca de t_detect
                closest_scan = None
                best_diff = float('inf')
                for scan_msg in self.scan_buffer:
                    t_scan = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
                    d = abs(t_scan - t_detect)
                    self.get_logger().debug(f"Timestamp difference: {d:.6f} seconds")
                    if d < best_diff:
                        best_diff = d
                        closest_scan = scan_msg

                max_diff = 0.3  # Permitir hasta x segundos de diferencia
                if best_diff > max_diff:
                    self.get_logger().warn(f"No LiDAR scan found within {max_diff} seconds of the detection timestamp.")
                    return

                if closest_scan is None:
                    closest_scan = self.scan_buffer[-1]  # Usar el escaneo más reciente
                    self.get_logger().warn("No close match found; using the most recent scan as fallback.")

                # Ajustar angle_rad al rango [angle_min, angle_max]
                wrapped_angle = (-angle_rad) % (2 * math.pi)
                epsilon = 1e-6
                if wrapped_angle > closest_scan.angle_max - epsilon:
                   wrapped_angle = 0.0   
                if wrapped_angle < closest_scan.angle_min or wrapped_angle > closest_scan.angle_max:
                    self.get_logger().warn(f"Angle {wrapped_angle:.2f} rad is outside the LiDAR scan range [{closest_scan.angle_min:.2f}, {closest_scan.angle_max:.2f}].")
                    return

                # Calcular el índice i para el ángulo ajustado
                i_wrapped = int(round((wrapped_angle - closest_scan.angle_min) / closest_scan.angle_increment))
                i_wrapped = max(0, min(i_wrapped, len(closest_scan.ranges) - 1))  # Asegurar que el índice esté dentro de los límites

                # Registros de depuración para los valores calculados
                self.get_logger().info(f"Wrapped angle: {wrapped_angle:.2f} rad --- {wrapped_angle * (180.0 / math.pi):.2f}°")
                self.get_logger().info(f"Computed wrapped index: {i_wrapped}, angle_min: {closest_scan.angle_min:.2f}, angle_max: {closest_scan.angle_max:.2f}, angle_increment: {closest_scan.angle_increment:.6f}")
                self.get_logger().info(f"Distance at wrapped index {i_wrapped}: {closest_scan.ranges[i_wrapped]:.2f}")

                # Obtener la distancia en el índice calculado
                distance = closest_scan.ranges[i_wrapped]
                if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                    self.get_logger().warn("LiDAR return invalid at that angle; optionally try the next closest scan.")
                    return

                # Comprobar si el objeto está dentro de 1.9 metros
                if distance > 1.5:
                    self.get_logger().info(f"Object detected at {distance:.2f} m, which exceeds the 1.9 m threshold. Clustering will not start.")
                    return


                # DETECCIÓN DE OBJETOS MEDIANTE AGRUPAMIENTO ----------------------------------------------
                import numpy as np
                from scipy.ndimage import label


                # Parámetros
                same_object_thr = self.name_object_thr  # Distancia mínima para considerar un objeto válido
                I_min = self.I_min  # Umbral mínimo de intensidad
                I_max = self.I_max
                bw_min = self.bw_min  # Ancho físico mínimo del clúster en metros
                bw_max = self.bw_max  # Ancho físico máximo del clúster en metros

                # Definir la zona de agrupamiento alrededor de i_wrapped
                cluster_radius = self.cluster_radius  # Número de índices a incluir a cada lado
                num_indexes = len(closest_scan.ranges)  # Número total de índices del LiDAR

                # Calcular el rango de índices a considerar, manejando el wrap-around
                start_index = (i_wrapped - cluster_radius) % num_indexes
                end_index = (i_wrapped + cluster_radius) % num_indexes

                if start_index < end_index:
                    zone_mask = np.zeros(num_indexes, dtype=bool)
                    zone_mask[start_index:end_index + 1] = True
                else:
                    # Caso con wrap-around: dividir en dos rangos
                    zone_mask = np.zeros(num_indexes, dtype=bool)
                    zone_mask[start_index:] = True
                    zone_mask[:end_index + 1] = True

                # Aplicar la máscara de zona a distancias e intensidades
                ranges_arr = np.array(closest_scan.ranges)
                intens_arr = np.array(closest_scan.intensities)
                valid_range_mask = np.isfinite(ranges_arr) & (ranges_arr > 0.1) & (ranges_arr <= 2.0)
                valid_intensity_mask = (intens_arr > I_min) & (intens_arr < I_max)

                
                if self.use_sim_time:
                    candidate_mask = valid_range_mask & zone_mask
                else:
                    candidate_mask = valid_range_mask & valid_intensity_mask & zone_mask


                # Máscara de similitud entre adyacentes dentro de la zona de agrupamiento
                deltas = np.abs(ranges_arr[:-1] - ranges_arr[1:]) <= same_object_thr
                adjacent_both_valid = candidate_mask[:-1] & candidate_mask[1:]
                cluster_mask_1d = deltas & adjacent_both_valid

                # Etiquetar secuencias conectadas
                labeled, num_clust = label(cluster_mask_1d)
                clusters = []
                for cid in range(1, num_clust + 1):
                    pos = np.where(labeled == cid)[0]
                    start = pos[0]
                    end = pos[-1] + 1
                    beam_indices = np.arange(start, end + 1)
                    clusters.append(beam_indices)

                # Filtrar clústeres con más de una lectura
                valid_clusters = [c for c in clusters if len(c) > 1]

                # Filtrar clústeres según el ancho físico
                filtered_clusters = []
                for c in valid_clusters:
                    start_idx = c[0]  # Primer índice del clúster
                    end_idx = c[-1]  # Último índice del clúster

                    # Calcular el ángulo barrido por el clúster
                    angular_span = (end_idx - start_idx) * closest_scan.angle_increment  # Ángulo en radianes

                    # Usar la distancia media del clúster para calcular el ancho físico
                    med_r = np.median(ranges_arr[c])  # Distancia media del clúster
                    cluster_width = 2 * med_r * math.sin(angular_span / 2)  # Ancho físico usando trigonometría

                    if bw_min <= cluster_width <= bw_max:
                        filtered_clusters.append(c)
                        self.get_logger().info(
                            f"Cluster at indices {start_idx}-{end_idx} with width {cluster_width:.2f} m "
                            f"accepted (within range [{bw_min}, {bw_max}])."
                        )
                    else:
                        self.get_logger().info(
                            f"Cluster at indices {start_idx}-{end_idx} with width {cluster_width:.2f} m "
                            f"filtered out (outside range [{bw_min}, {bw_max}])."
                        )

                valid_clusters = filtered_clusters

                if not valid_clusters:
                    self.get_logger().warn("No valid clusters found. Exiting object detection process.")
                    return  # Salir del método

                # Buscar el clúster más cercano al índice del objeto rojo detectado y al robot
                best_cluster = None
                best_score = float('inf')  # Un valor menor es mejor

                for c in valid_clusters:
                    center_idx = int(round(np.mean(c)))  # Índice central del clúster
                    distance = ranges_arr[center_idx]  # Distancia del centro del clúster

                    # Fórmula de puntuación ajustada:
                    # - Penaliza clústeres más alejados del robot
                    # - Penaliza clústeres más alejados del objeto rojo detectado
                    proximity_penalty = abs(center_idx - i_wrapped)  # Penalizar según la distancia a i_wrapped
                    score = distance + 0.5 * proximity_penalty  # La distancia pesa más que la penalización

                    self.get_logger().info(
                        f"Cluster center index: {center_idx}, Distance: {distance:.2f}, "
                        f"Proximity penalty: {proximity_penalty:.2f}, Score: {score:.2f}"
                    )

                    if score < best_score:
                        best_score = score
                        best_cluster = c

                # Usar el mejor clúster
                if best_cluster is not None:
                    i_object = int(round(np.mean(best_cluster)))  # Índice central del mejor clúster
                    distance = ranges_arr[i_object]

                    # Ángulo final
                    final_angle = closest_scan.angle_min + i_object * closest_scan.angle_increment
                else:
                    self.get_logger().warn("No suitable cluster found. Exiting object detection process.")
                    return

                # Registro de depuración para el ángulo final
                self.get_logger().info(f"Final angle for object: {final_angle:.2f} rad --- {final_angle * (180.0 / math.pi):.2f}°")
                self.get_logger().info(f"Final index: {i_object}")
                self.get_logger().info(f"Distance final index {i_object}: {distance:.2f}")

                # Llamar a marker_creation
                self.marker_creation(final_angle, t_detect, distance)

        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Error parsing message: {msg.data}. Exception: {e}')

    def scan_callback(self, scan_msg: LaserScan):
        """
        Almacenar en buffer los 20 mensajes de LaserScan más recientes.
        """
        self.scan_buffer.append(scan_msg)
        #self.get_logger().debug(f"Escaneo LiDAR almacenado en {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}, total almacenado: {len(self.scan_buffer)}")
        #self.get_logger().info(f"Escaneo LiDAR con {len(scan_msg.ranges)} mediciones almacenadas")

def main(args=None):
    rclpy.init(args=args)
    node = ExploreController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

