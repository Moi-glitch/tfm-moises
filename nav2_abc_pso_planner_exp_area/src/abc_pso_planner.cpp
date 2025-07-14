#include "nav2_abc_pso_planner_exp_area/abc_pso_planner.hpp"  // Header del planner ABC+PSO
#include <random>       // Para generación de números aleatorios
#include <algorithm>    // Para std::max, std::min
#include <limits>       // Para std::numeric_limits
#include <cmath>        // Para std::hypot, std::atan2
#include <numeric>      // Para std::accumulate
#include "nav2_util/line_iterator.hpp"  // Para recorrer celdas de un segmento
#include "tf2/utils.h"  // Para tf2::getYaw

namespace nav2_abc_pso_planner  // Espacio de nombres del plugin
{


// Constructor: inicializa el logger con el nombre "ABCPSOPlanner"
ABCPSOPlanner::ABCPSOPlanner()
  : logger_(rclcpp::get_logger("ABCPSOPlanner")),  // Obtiene logger ROS 2
    cached_best_cost_(std::numeric_limits<double>::infinity())
{
}

// Destructor: no necesita liberar recursos específicos
ABCPSOPlanner::~ABCPSOPlanner() {}

// Configura el planner con parámetros iniciales y punteros necesarios
void ABCPSOPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  costmap_ = costmap_ros;
  auto node = parent.lock();
  logger_ = node->get_logger();

  // Declare and retrieve parameters
  robot_radius_ = node->declare_parameter<double>(name + ".robot_radius", 0.1);
  generation_radius_ =
    node->declare_parameter<double>(name + ".generation_radius", 0.0);
  swarm_size_ = node->declare_parameter<int>(name + ".swarm_size", 100);
  max_iterations_ = node->declare_parameter<int>(name + ".max_iterations", 100);
  num_waypoints_ = node->declare_parameter<int>(name + ".num_waypoints", 7);  // Default: 7

  // PSO-specific parameters
  w_ = node->declare_parameter<double>(name + ".w", 0.9);
  wdamp_ = node->declare_parameter<double>(name + ".wdamp", 0.99);
  c1_ = node->declare_parameter<double>(name + ".c1", 2.0);
  c2_ = node->declare_parameter<double>(name + ".c2", 1.3);

  w_length_ = node->declare_parameter<double>(name + ".w_length", 1.0);
  w_offmap_ = node->declare_parameter<double>(name + ".w_offmap", 10.0);
  w_cell_ = node->declare_parameter<double>(name + ".w_cell", 1.0);
  w_unknown_ = node->declare_parameter<double>(name + ".w_unknown", 5.0);
  w_lethal_ = node->declare_parameter<double>(name + ".w_lethal", 100.0);
  w_inflated_ = node->declare_parameter<double>(name + ".w_inflated", 20.0);
  w_turn_ = node->declare_parameter<double>(name + ".w_turn", 1.0);

  RCLCPP_INFO(logger_,
    "Configured ABCPSOPlanner with num_waypoints=%d, w=%.2f, wdamp=%.2f, c1=%.2f, c2=%.2f, swarm_size=%d, max_iterations=%d, w_length=%.2f, w_offmap=%.2f, w_cell=%.2f, w_unknown=%.2f, w_lethal=%.2f, w_inflated=%.2f, w_turn=%.2f, generation_radius=%.2f",
    num_waypoints_, w_, wdamp_, c1_, c2_, swarm_size_, max_iterations_, w_length_, w_offmap_, w_cell_, w_unknown_, w_lethal_, w_inflated_, w_turn_, generation_radius_);
}

// Hooks del ciclo de vida sin uso adicional
void ABCPSOPlanner::cleanup() {}
void ABCPSOPlanner::activate() {}
void ABCPSOPlanner::deactivate() {}

// Selección por ruleta para onlooker bees: elige índice según vector de probabilidades P
static int RouletteWheelSelection(const std::vector<double> & P)
{
  static std::mt19937 gen(std::random_device{}());  // Generador único
  std::uniform_real_distribution<> dist(0.0, 1.0);  // Distribución uniforme [0,1]
  double r = dist(gen);                             // Valor aleatorio
  double cum_sum = 0.0;                             // Suma acumulada

  // Bucle para encontrar primer índice donde la suma excede r
  for (size_t i = 0; i < P.size(); ++i) {
    cum_sum += P[i];  // Acumula probabilidad
    if (cum_sum >= r) {
      return static_cast<int>(i);
    }
  }
  return static_cast<int>(P.size() - 1);  // Fallback: último índice
}

// Método principal: combina ABC y PSO para generar un nav_msgs::msg::Path
// Para evitar jitter en RViz, cachea el plan si start y goal no cambian
nav_msgs::msg::Path ABCPSOPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,  // Punto inicial
  const geometry_msgs::msg::PoseStamped & goal)   // Punto destino
{
  if (goal.pose.position.x != prev_goal_.pose.position.x ||
      goal.pose.position.y != prev_goal_.pose.position.y) {
    cached_best_cost_ = std::numeric_limits<double>::infinity();
    cached_path_.poses.clear();
    prev_goal_ = goal;
  }

  // Guarda los puntos para evaluación de colisiones
  current_start_ = start;
  current_goal_ = goal;
  // Determina número de variables (2*num_waypoints) basado en posición inicializada
  int nVar = static_cast<int>(initializeSwarm(start, goal)[0].position.size());
  int nPop = swarm_size_;           // Población
  int MaxIt = max_iterations_;      // Iteraciones
  int nOnlooker = nPop;             // Onlooker bees = nPop
  int L = static_cast<int>(std::round(0.6 * nVar * nPop));  // Abandonment limit

  // Parámetros PSO
  double w = w_, wdamp = wdamp_;      // Peso inercial y damping
  double c1 = c1_, c2 = c2_;         // Coeficientes cognitivo y social

  // Límites de búsqueda: recta entre start y goal
  double x0 = start.pose.position.x;
  double y0 = start.pose.position.y;
  double x1 = goal.pose.position.x;
  double y1 = goal.pose.position.y;
  double xmin = std::min(x0, x1) - generation_radius_;
  double xmax = std::max(x0, x1) + generation_radius_;
  double ymin = std::min(y0, y1) - generation_radius_;
  double ymax = std::max(y0, y1) + generation_radius_;

  // Inicializa generador y distribuciones
  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<> phi_dist(-1.0, 1.0);  // Para ABC φ
  std::uniform_real_distribution<> uni01(0.0, 1.0);     // Para PSO r1,r2
  std::uniform_int_distribution<> idx_dist(0, nPop - 1); // Índices aleatorios
  std::uniform_real_distribution<> ux(xmin, xmax);      // Scout bees x
  std::uniform_real_distribution<> uy(ymin, ymax);      // Scout bees y

  // Inicialización del enjambre con posiciones y velocidades iniciales
  std::vector<BeeParticle> swarm = initializeSwarm(start, goal);  // Llamada a init
  std::vector<int> C(nPop, 0);              // Contadores de scouts
  std::vector<double> best_path;            // Guarda mejor ruta global
  double best_cost = std::numeric_limits<double>::max();  // Coste global mínimo

  // Bucle de optimización ABC + PSO
  for (int it = 0; it < MaxIt; ++it) {
    // --------- ABC: Employed Bees ---------
    for (int i = 0; i < nPop; ++i) {
      // Selecciona aleatoriamente otra abeja k != i
      int k;
      do {
        k = idx_dist(gen);
      } while (k == i);

      // Copia posiciones de abeja i (xi) y k (xk)
      const auto xi = swarm[i].position;
      const auto xk = swarm[k].position;
      std::vector<double> newpos = xi;  // Partida desde xi

      // Genera nueva posición: xi[d] + φ*(xi[d] - xk[d])
      for (int d = 0; d < nVar; ++d) {
        double phi = phi_dist(gen);
        double val = xi[d] + phi * (xi[d] - xk[d]);
        // Aplica límites
        double lo = (d % 2 == 0 ? xmin : ymin);
        double hi = (d % 2 == 0 ? xmax : ymax);
        newpos[d] = std::max(lo, std::min(val, hi));
      }

      // Evalúa coste y decide si actualizar la abeja i
      double newcost = evaluateCost(newpos);
      if (newcost < swarm[i].cost) {
        swarm[i].position = newpos;
        swarm[i].cost = newcost;
        C[i] = 0;  // Reset de contador
      } else {
        C[i]++;    // Incrementa contador de no-mejora
      }
    }

    // --------- ABC: Fitness y Probabilidades ---------
    std::vector<double> fitness(nPop);
    double meanCost = 0.0;
    for (auto & p : swarm) {
      meanCost += p.cost;  // Suma costes
    }
    meanCost /= nPop;       // Calcula media
    for (int i = 0; i < nPop; ++i) {
      // Fitness = exp(-cost / meanCost)
      fitness[i] = std::exp(-swarm[i].cost / meanCost);
    }
    double sumF = std::accumulate(fitness.begin(), fitness.end(), 0.0);
    std::vector<double> prob(nPop);
    for (int i = 0; i < nPop; ++i) {
      prob[i] = fitness[i] / sumF;  // Normaliza
    }

    // --------- ABC: Onlooker Bees ---------
    for (int m = 0; m < nOnlooker; ++m) {
      int i = RouletteWheelSelection(prob);  // Selección por ruleta
      int k;
      do {
        k = idx_dist(gen);
      } while (k == i);

      const auto xi = swarm[i].position;
      const auto xk = swarm[k].position;
      std::vector<double> newpos = xi;
      for (int d = 0; d < nVar; ++d) {
        double phi = phi_dist(gen);
        double val = xi[d] + phi * (xi[d] - xk[d]);
        double lo = (d % 2 == 0 ? xmin : ymin);
        double hi = (d % 2 == 0 ? xmax : ymax);
        newpos[d] = std::max(lo, std::min(val, hi));
      }
      double newcost = evaluateCost(newpos);
      if (newcost < swarm[i].cost) {
        swarm[i].position = newpos;
        swarm[i].cost = newcost;
        C[i] = 0;
      } else {
        C[i]++;
      }
    }

    // --------- ABC: Scout Bees ---------
    for (int i = 0; i < nPop; ++i) {
      if (C[i] >= L) {  // Si supera el límite, reinicia aleatoriamente
        std::vector<double> rnd(nVar);
        for (int d = 0; d < nVar; ++d) {
          rnd[d] = (d % 2 == 0 ? ux(gen) : uy(gen));
        }
        swarm[i].position = rnd;
        swarm[i].cost = evaluateCost(rnd);
        C[i] = 0;
      }
    }

    // --------- PSO: Prepara mejor global de ABC ---------
    std::vector<double> global_best;
    double bestCostABC = std::numeric_limits<double>::max();
    for (auto & p : swarm) {
      if (p.cost < bestCostABC) {
        bestCostABC = p.cost;
        global_best = p.position;
      }
    }

    // --------- PSO: Actualiza partículas ---------
    for (int i = 0; i < nPop; ++i) {
      auto & p = swarm[i];
      for (int d = 0; d < nVar; ++d) {
        // Velocidad: w*vel + c1*r1*(pbest-x) + c2*r2*(gbest-x)
        double r1 = uni01(gen);
        double r2 = uni01(gen);
        p.velocity[d] =
          w * p.velocity[d] +
          c1 * r1 * (p.best_position[d] - p.position[d]) +
          c2 * r2 * (global_best[d] - p.position[d]);
        // Actualiza posición y aplica límites
        double val = p.position[d] + p.velocity[d];
        double lo = (d % 2 == 0 ? xmin : ymin);
        double hi = (d % 2 == 0 ? xmax : ymax);
        p.position[d] = std::max(lo, std::min(val, hi));
      }
      // Evalúa y actualiza mejor personal
      double newcost = evaluateCost(p.position);
      if (newcost < p.cost) {
        p.best_position = p.position;
        p.cost = newcost;
      }
    }
    w *= wdamp;  // Reduce inercia por damping

    // --------- Selecciona mejor ruta PSO ---------
    double bestCostPSO = std::numeric_limits<double>::max();
    std::vector<double> bestPathPSO;
    for (auto & p : swarm) {
      if (p.cost < bestCostPSO) {
        bestCostPSO = p.cost;
        bestPathPSO = p.position;
      }
    }

    // Imprime estado actual
    RCLCPP_INFO(logger_, "Iter %d: ABC=%.3f, PSO=%.3f", it, bestCostABC, bestCostPSO);

    // Actualiza mejor global final
    if (bestCostPSO < best_cost) {
      best_cost = bestCostPSO;
      best_path = bestPathPSO;
    }
  }

  // Convierte y devuelve la mejor ruta en formato ROS
  // Convierte intermediate waypoints a nav_msgs::msg::Path
nav_msgs::msg::Path ros_path = convertToNavPath(best_path);
// Añade pose inicial
geometry_msgs::msg::PoseStamped start_pose = start;
start_pose.header = ros_path.header;
ros_path.poses.insert(ros_path.poses.begin(), start_pose);
// Añade pose objetivo
geometry_msgs::msg::PoseStamped goal_pose = goal;
goal_pose.header = ros_path.header;
  ros_path.poses.push_back(goal_pose);

  if (std::isfinite(cached_best_cost_) &&
      best_cost >= cached_best_cost_ && !cached_path_.poses.empty()) {
    RCLCPP_INFO(logger_, "Reutilizando ruta cacheada coste=%.3f nueva=%.3f", cached_best_cost_, best_cost);

    nav_msgs::msg::Path trimmed = cached_path_;
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < trimmed.poses.size(); ++i) {
      double dx = trimmed.poses[i].pose.position.x - start.pose.position.x;
      double dy = trimmed.poses[i].pose.position.y - start.pose.position.y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    if (closest_idx > 0) {
      trimmed.poses.erase(trimmed.poses.begin(), trimmed.poses.begin() + closest_idx);
    }

    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header = trimmed.header;
    if (!trimmed.poses.empty()) {
      auto & next = trimmed.poses.front().pose.position;
      double dx = next.x - start.pose.position.x;
      double dy = next.y - start.pose.position.y;
      double yaw = std::atan2(dy, dx);
      start_pose.pose.orientation.z = std::sin(yaw * 0.5);
      start_pose.pose.orientation.w = std::cos(yaw * 0.5);
    } else {
      start_pose.pose.orientation.w = 1.0;
    }
    trimmed.poses.insert(trimmed.poses.begin(), start_pose);
    return trimmed;
  }

  cached_best_cost_ = best_cost;
  cached_path_ = ros_path;
  return ros_path;
}

// Inicializa enjambre con posiciones y velocidades aleatorias
std::vector<ABCPSOPlanner::BeeParticle> ABCPSOPlanner::initializeSwarm(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<BeeParticle> swarm;
  int num_waypoints = num_waypoints_;  // Numero de puntos intermedios

  double x0 = start.pose.position.x;
  double y0 = start.pose.position.y;
  double x1 = goal.pose.position.x;
  double y1 = goal.pose.position.y;
  double xmin = std::min(x0, x1) - generation_radius_;
  double xmax = std::max(x0, x1) + generation_radius_;
  double ymin = std::min(y0, y1) - generation_radius_;
  double ymax = std::max(y0, y1) + generation_radius_;

  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<> x_dist(xmin, xmax);
  std::uniform_real_distribution<> y_dist(ymin, ymax);
  std::uniform_real_distribution<> vel_dist(-0.5, 0.5); // Velocidad inicial

  // Crea cada partícula con interpolación + jitter
  for (int i = 0; i < swarm_size_; ++i) {
    BeeParticle particle;
    particle.position.resize(num_waypoints * 2);
    particle.velocity.resize(num_waypoints * 2);
    particle.best_position.resize(num_waypoints * 2);

    for (int j = 0; j < num_waypoints; ++j) {
      double xi, yi;
      unsigned int mx, my;
      int attempts = 0;
      do {
        xi = x_dist(gen);
        yi = y_dist(gen);
        attempts++;
      } while (
        attempts < 100 &&
        (!costmap_->getCostmap()->worldToMap(xi, yi, mx, my) ||
        costmap_->getCostmap()->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE));

      particle.position[2 * j] = xi;
      particle.position[2 * j + 1] = yi;
      particle.velocity[2 * j] = vel_dist(gen);
      particle.velocity[2 * j + 1] = vel_dist(gen);
      particle.best_position[2 * j] = xi;
      particle.best_position[2 * j + 1] = yi;
    }
    // Calcula coste inicial de la partícula
    particle.cost = evaluateCost(particle.position);
    swarm.push_back(particle);
  }
  return swarm;
}

// Evalúa coste recorriendo cada celda de los segmentos con Bresenham.
// Para cada celda visitada se suma su costo. Las celdas sin información se
// penalizan moderadamente para permitir explorar zonas desconocidas, mientras
// que las letales se penalizan fuertemente y detienen la evaluación.
// En abc_pso_planner.cpp:
double ABCPSOPlanner::evaluateCost(const std::vector<double> & path)
{
  double total_cost = 0.0;
  double resolution = costmap_->getCostmap()->getResolution();
  double inv_res = 1.0 / resolution;  // para convertir metros ↔ celdas
  int radius_cells = static_cast<int>(std::ceil(robot_radius_ * inv_res));
  unsigned int size_x = costmap_->getCostmap()->getSizeInCellsX();
  unsigned int size_y = costmap_->getCostmap()->getSizeInCellsY();
  int n = static_cast<int>(path.size() / 2);

  // Construye lista completa de puntos incluyendo inicio y objetivo
  std::vector<std::pair<double, double>> pts;
  pts.emplace_back(current_start_.pose.position.x, current_start_.pose.position.y);
  for (int i = 0; i < n; ++i) {
    pts.emplace_back(path[2 * i], path[2 * i + 1]);
  }
  pts.emplace_back(current_goal_.pose.position.x, current_goal_.pose.position.y);

  // Recorre cada segmento
  double goal_yaw = tf2::getYaw(current_goal_.pose.orientation);
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double x0 = pts[i].first,  y0 = pts[i].second;
    double x1 = pts[i+1].first, y1 = pts[i+1].second;

    double yaw = std::atan2(y1 - y0, x1 - x0);
    double diff = yaw - goal_yaw;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    total_cost += w_turn_ * std::fabs(diff) * 180.0 / M_PI;

    // 1) Distancia (convertida a celdas)
    double dist_m = std::hypot(x1 - x0, y1 - y0);
    total_cost += w_length_ * dist_m * inv_res;

    unsigned int mx0, my0, mx1, my1;
    bool in0 = costmap_->getCostmap()->worldToMap(x0, y0, mx0, my0);
    bool in1 = costmap_->getCostmap()->worldToMap(x1, y1, mx1, my1);
    if (!in0 || !in1) {
      // Penalización proporcional a la longitud fuera de mapa
      total_cost += w_offmap_ * (dist_m * inv_res);
      continue;
    }

    // 2) Recorre celdas del segmento
    for (nav2_util::LineIterator line(mx0, my0, mx1, my1);
         line.isValid(); line.advance())
    {
      unsigned int lx = line.getX(), ly = line.getY();
      unsigned char cell = costmap_->getCostmap()->getCost(lx, ly);

      // 2.a) coste de celda normalizado
      double cell_norm = static_cast<double>(cell) / 255.0;
      total_cost += w_cell_ * cell_norm;

      // 2.b) casos especiales
      if (cell == nav2_costmap_2d::NO_INFORMATION) {
        total_cost += w_unknown_;
      } else if (cell >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        total_cost += w_lethal_;
      }

      // 3) clearance por radio: aquí solo inflado
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          if (dx == 0 && dy == 0) continue;
          int nx = static_cast<int>(lx) + dx;
          int ny = static_cast<int>(ly) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x)
            || ny >= static_cast<int>(size_y)) continue;
          unsigned char neigh = costmap_->getCostmap()->getCost(nx, ny);
          if (neigh >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            total_cost += w_lethal_;
          } else if (neigh >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            // suma sólo el peso inflado (no el valor bruto)
            total_cost += w_inflated_;
          }
        }
      }
    }
  }

  return total_cost;
}

// Convierte vector<double> en nav_msgs::msg::Path
nav_msgs::msg::Path ABCPSOPlanner::convertToNavPath(const std::vector<double> & path)
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = costmap_->getGlobalFrameID();
  ros_path.header.stamp = rclcpp::Clock().now();
  int n = static_cast<int>(path.size() / 2);

  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;
    pose.pose.position.x = path[2*i];
    pose.pose.position.y = path[2*i + 1];
    pose.pose.position.z = 0.0;
    // Calcula orientación hacia siguiente punto
    if (i < n - 1) {
      double dx = path[2*(i+1)] - path[2*i];
      double dy = path[2*(i+1) + 1] - path[2*i + 1];
      double yaw = std::atan2(dy, dx);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    } else if (!ros_path.poses.empty()) {
      // Conserva última orientación
      pose.pose.orientation = ros_path.poses.back().pose.orientation;
    } else {
      // Identidad si es único punto
      pose.pose.orientation.w = 1.0;
    }
    ros_path.poses.push_back(pose);
  }
  return ros_path;
}

}  // namespace nav2_abc_pso_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_abc_pso_planner::ABCPSOPlanner, nav2_core::GlobalPlanner)

