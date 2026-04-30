# kiva_warehouse_optimized.py
"""
Amazon Kiva Robot Симуляци - Роботууд давхцалгүй ажиллах
Advanced Multi-Agent Coordination System
"""

import pygame
import sys
import math
import heapq
import random
from collections import deque, defaultdict, namedtuple
import numpy as np

# ========== ТОГТМОЛ УТГУУД ==========
SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 900
GRID_WIDTH = 40
GRID_HEIGHT = 25
CELL_SIZE = 30

# Системийн тохиргоо
NUM_ROBOTS = 12
NUM_PODS = 80
NUM_PICK_STATIONS = 6
NUM_CHARGING_STATIONS = 6
MAX_BATTERY = 100
BATTERY_THRESHOLD = 25
ROBOT_SPEED = 2

# Өнгөнүүд
COLORS = {
    'WHITE': (255, 255, 255),
    'BLACK': (0, 0, 0),
    'LIGHT_GREY': (240, 240, 240),
    'DARK_GREY': (120, 120, 120),
    'ROBOT_IDLE': (65, 105, 225),
    'ROBOT_BUSY': (220, 20, 60),
    'ROBOT_CHARGING': (30, 144, 255),
    'POD_STORED': (184, 134, 11),
    'POD_CARRIED': (210, 105, 30),
    'PICK_STATION': (60, 179, 113),
    'CHARGING_STATION': (255, 215, 0),
    'PATH': (255, 182, 193),
    'RESERVED': (255, 200, 200, 80),
    'OBSTACLE': (47, 79, 79),
    'TEXT': (50, 50, 50),
    'GRID_LINE': (200, 200, 200),
    'SUCCESS': (34, 139, 34),
    'WARNING': (255, 165, 0),
    'ERROR': (220, 20, 60)
}

# Цэнэглэх станцууд
CHARGING_STATIONS = [
    (1, 1), (1, GRID_HEIGHT-2), (GRID_WIDTH-2, 1), 
    (GRID_WIDTH-2, GRID_HEIGHT-2), (GRID_WIDTH//2, 1), (GRID_WIDTH//2, GRID_HEIGHT-2)
]

Task = namedtuple("Task", ["id", "pod_id", "pick_station_id", "status", "items", "assigned_robot"])

class PathNode:
    __slots__ = ('position', 'g_cost', 'h_cost', 'parent', 'time_step')
    
    def __init__(self, position, g_cost, h_cost, parent=None, time_step=0):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent
        self.time_step = time_step
    
    @property
    def f_cost(self):
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

def manhattan_distance(pos1, pos2):
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def get_neighbors(position):
    """4 чиглэлд хөршүүдийг буцаах"""
    x, y = position
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    
    neighbors = []
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
            neighbors.append((nx, ny))
    return neighbors

class CollisionAvoidanceSystem:
    """Мөргөлдөөнөөс зайлсхийх төв систем"""
    
    def __init__(self, world):
        self.world = world
        self.robot_paths = {}  # robot_id -> [path]
        self.reservation_table = defaultdict(dict)  # (x,y,time) -> robot_id
        self.conflict_history = defaultdict(int)
    
    def reserve_path(self, robot_id, path, start_time):
        """Роботын замыг бүхэлд нь захиалах"""
        for i, cell in enumerate(path):
            time_step = start_time + i
            self.reservation_table[(cell[0], cell[1], time_step)] = robot_id
            self.world.add_time_reservation(cell, time_step, robot_id)
    
    def is_path_clear(self, robot_id, path, start_time):
        """Зам чөлөөтэй эсэхийг шалгах"""
        for i, cell in enumerate(path):
            time_step = start_time + i
            
            # Бусад роботуудын захиалга шалгах
            if (cell[0], cell[1], time_step) in self.reservation_table:
                other_robot = self.reservation_table[(cell[0], cell[1], time_step)]
                if other_robot != robot_id:
                    return False
            
            # Тогтмол саадууд шалгах
            if not self.world.is_cell_available(cell, robot_id, time_step):
                return False
        
        return True
    
    def clear_robot_reservations(self, robot_id):
        """Роботын захиалгуудыг цэвэрлэх"""
        to_remove = []
        for key, rid in self.reservation_table.items():
            if rid == robot_id:
                to_remove.append(key)
        for key in to_remove:
            del self.reservation_table[key]

class WarehouseGrid:
    def __init__(self):
        self.width = GRID_WIDTH
        self.height = GRID_HEIGHT
        self.obstacles = set()
        self.pick_stations = []
        self.charging_stations = CHARGING_STATIONS.copy()
        
        self.cell_reservations = {}
        self.time_reservations = defaultdict(dict)
        self.current_time = 0
        self.pod_locations = {}
        
        self.collision_system = CollisionAvoidanceSystem(self)
        self.generate_warehouse_layout()
    
    def generate_warehouse_layout(self):
        """Агуулахын бүтэц үүсгэх"""
        # Хана
        for x in range(self.width):
            self.obstacles.add((x, 0))
            self.obstacles.add((x, self.height-1))
        for y in range(self.height):
            self.obstacles.add((0, y))
            self.obstacles.add((self.width-1, y))
        
        # Pick станцууд
        for i in range(NUM_PICK_STATIONS):
            x = self.width - 3
            y = 2 + i * 3
            if y < self.height - 2:
                self.pick_stations.append((x, y, i))
        
        # Дотоод саадууд
        for x in range(5, self.width-5, 6):
            for y in range(4, self.height-4, 2):
                if random.random() < 0.3:
                    self.obstacles.add((x, y))
        
        # Pod байршил
        self.generate_pod_locations()
    
    def generate_pod_locations(self):
        pod_id = 1
        storage_areas = [
            (2, 3, self.width-10, self.height//2 - 2),
            (2, self.height//2 + 2, self.width-10, self.height-4)
        ]
        
        for area in storage_areas:
            x_start, y_start, x_end, y_end = area
            for x in range(x_start, x_end, 2):
                for y in range(y_start, y_end, 2):
                    if pod_id > NUM_PODS:
                        return
                    pos = (x, y)
                    if (pos not in self.obstacles and 
                        pos not in self.pod_locations.values()):
                        self.pod_locations[pod_id] = pos
                        pod_id += 1
    
    def update_time(self):
        self.current_time += 1
    
    def reserve_cell(self, cell, robot_id, duration=3):
        self.cell_reservations[cell] = (robot_id, self.current_time + duration)
    
    def add_time_reservation(self, cell, time_step, robot_id):
        self.time_reservations[(cell, time_step)] = robot_id
    
    def is_cell_available(self, cell, robot_id, time_step=None):
        if cell in self.obstacles:
            return False
        
        if cell in self.cell_reservations:
            reserved_robot, end_time = self.cell_reservations[cell]
            if reserved_robot != robot_id and end_time > self.current_time:
                return False
        
        if time_step is not None:
            reserved_robot = self.time_reservations.get((cell, time_step))
            if reserved_robot and reserved_robot != robot_id:
                return False
        
        return True
    
    def get_pick_station_position(self, station_id):
        for x, y, sid in self.pick_stations:
            if sid == station_id:
                return (x, y)
        return None
    
    def find_empty_storage_location(self):
        storage_areas = [
            (2, 3, self.width-10, self.height//2 - 2),
            (2, self.height//2 + 2, self.width-10, self.height-4)
        ]
        
        for area in storage_areas:
            x_start, y_start, x_end, y_end = area
            for x in range(x_start, x_end, 2):
                for y in range(y_start, y_end, 2):
                    pos = (x, y)
                    if (self.is_cell_available(pos, None) and 
                        pos not in self.pod_locations.values()):
                        return pos
        return None

def cooperative_a_star(start, goal, world, robot_id, other_robots_positions, max_time=100):
    """Хамтын A* алгоритм - бусад роботуудын байрлалыг харгалзан"""
    if start == goal:
        return [start]
    
    open_set = []
    start_node = PathNode(start, 0, manhattan_distance(start, goal))
    heapq.heappush(open_set, (start_node.f_cost, id(start_node), start_node))
    
    came_from = {}
    g_score = {start: 0}
    best_nodes = {start: start_node}
    
    while open_set:
        _, _, current = heapq.heappop(open_set)
        
        if current.position == goal:
            path = []
            node = current
            while node is not None:
                path.append(node.position)
                node = node.parent
            return path[::-1]
        
        if current.time_step > max_time:
            continue
            
        for neighbor in get_neighbors(current.position):
            # Боломжгүй нүд шалгах
            if not world.is_cell_available(neighbor, robot_id, current.time_step + 1):
                continue
            
            # Бусад роботуудын ирээдүйн байрлал шалгах
            collision_risk = False
            for other_id, other_pos in other_robots_positions.items():
                if other_id != robot_id:
                    # Бусад роботуудын хөдөлгөөнийг урьдчилан таамаглах
                    if neighbor == other_pos:
                        collision_risk = True
                        break
            
            if collision_risk:
                continue
            
            tentative_g = current.g_cost + 1
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                new_node = PathNode(
                    neighbor, 
                    tentative_g, 
                    manhattan_distance(neighbor, goal),
                    current,
                    current.time_step + 1
                )
                
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                best_nodes[neighbor] = new_node
                
                heapq.heappush(open_set, (new_node.f_cost, id(new_node), new_node))
    
    return None

class StoragePod:
    def __init__(self, pod_id, x, y):
        self.id = pod_id
        self.x = x
        self.y = y
        self.items = [f"item_{random.randint(1000, 9999)}" for _ in range(random.randint(3, 8))]
        self.is_carried = False
        self.carried_by = None
        self.color = COLORS['POD_STORED']
    
    def remove_items(self, items_to_remove):
        remaining_items = []
        removed_items = []
        
        for item in self.items:
            if item in items_to_remove:
                removed_items.append(item)
            else:
                remaining_items.append(item)
        
        self.items = remaining_items
        return removed_items
    
    def update_position(self, x, y):
        self.x = x
        self.y = y

class KivaRobot:
    def __init__(self, robot_id, start_x, start_y, world):
        self.id = robot_id
        self.x = start_x
        self.y = start_y
        self.world = world
        self.carrying_pod = None
        
        self.path = deque()
        self.state = "charging"
        self.goal_position = None
        
        self.battery = MAX_BATTERY
        self.speed = ROBOT_SPEED
        self.tick_counter = 0
        
        self.current_task = None
        self.wait_counter = 0
        self.max_wait_time = 8
        self.priority = 1
        
        self.color = COLORS['ROBOT_CHARGING']
        
        self.tasks_completed = 0
        self.distance_traveled = 0
        self.total_pods_carried = 0
        self.collision_count = 0
    
    @property
    def pos(self):
        return (self.x, self.y)
    
    def update_priority(self):
        self.priority = 1
        if self.carrying_pod:
            self.priority += 3
        if self.battery < BATTERY_THRESHOLD + 15:
            self.priority += 2
        if self.wait_counter > self.max_wait_time // 2:
            self.priority += 1
    
    def get_other_robots_positions(self, all_robots):
        """Бусад роботуудын байрлалыг авах"""
        positions = {}
        for robot in all_robots:
            if robot.id != self.id:
                positions[robot.id] = robot.pos
        return positions
    
    def plan_path_to(self, target_pos, all_robots):
        """Шинэ зам төлөвлөх - бусад роботуудын байрлалыг харгалзан"""
        self.path.clear()
        
        other_positions = self.get_other_robots_positions(all_robots)
        
        path = cooperative_a_star(
            self.pos, 
            target_pos, 
            self.world, 
            self.id,
            other_positions
        )
        
        if path:
            self.path = deque(path[1:])
            self.goal_position = target_pos
            
            # Замыг collision system-д бүртгэх
            full_path = [self.pos] + list(self.path)
            self.world.collision_system.reserve_path(self.id, full_path, self.world.current_time)
            
            return True
        
        return False
    
    def move_step(self, all_robots):
        """Нэг алхам хөдөлгөөн - бүх роботуудын мэдээлэлтэй"""
        if not self.path:
            return False
        
        next_cell = self.path[0]
        
        # Дараагийн нүд рүү хөдөлж болох эсэхийг шалгах
        can_move = True
        
        # Бүх роботуудын одоогийн байрлал шалгах
        for robot in all_robots:
            if robot.id != self.id and robot.pos == next_cell:
                can_move = False
                self.collision_count += 1
                break
        
        # Нүдний боломж шалгах
        if not self.world.is_cell_available(next_cell, self.id, self.world.current_time + 1):
            can_move = False
        
        if can_move:
            # Хөдөлгөөн хийх
            old_pos = self.pos
            self.world.reserve_cell(next_cell, self.id)
            self.world.reserve_cell(old_pos, self.id, 1)
            
            self.x, self.y = next_cell
            self.path.popleft()
            self.distance_traveled += 1
            
            if self.carrying_pod:
                self.carrying_pod.update_position(self.x, self.y)
            
            self.battery = max(0, self.battery - 0.5)
            self.wait_counter = 0
            self.color = COLORS['ROBOT_BUSY'] if self.carrying_pod else COLORS['ROBOT_IDLE']
            return True
        else:
            self.wait_counter += 1
            
            # Хэт урт хүлээсэн бол шинэ зам төлөвлөх
            if self.wait_counter > self.max_wait_time and self.goal_position:
                self.plan_path_to(self.goal_position, all_robots)
                self.wait_counter = 0
            
            return False
    
    def execute_task_sequence(self, all_robots):
        if not self.current_task:
            return
        
        task = self.current_task
        
        if self.state == "moving_to_pod" and not self.path:
            pod_pos = self.world.pod_locations.get(task.pod_id)
            if pod_pos == self.pos:
                self.state = "at_pod"
            else:
                self.state = "idle"
                self.current_task = None
        
        elif self.state == "at_pod":
            if self.pickup_pod(task.pod_id):
                pick_station_pos = self.world.get_pick_station_position(task.pick_station_id)
                if pick_station_pos and self.plan_path_to(pick_station_pos, all_robots):
                    self.state = "moving_to_station"
                else:
                    self.drop_pod()
                    self.state = "idle"
                    self.current_task = None
            else:
                self.state = "idle"
                self.current_task = None
        
        elif self.state == "moving_to_station" and not self.path:
            self.state = "at_station"
        
        elif self.state == "at_station":
            if self.carrying_pod and self.carrying_pod.id == task.pod_id:
                items_to_pick = task.items[:2]
                picked_items = self.carrying_pod.remove_items(items_to_pick)
                
                storage_pos = self.world.find_empty_storage_location()
                if storage_pos and self.plan_path_to(storage_pos, all_robots):
                    self.state = "returning_pod"
                else:
                    self.state = "idle"
                    self.current_task = None
        
        elif self.state == "returning_pod" and not self.path:
            if self.drop_pod():
                self.tasks_completed += 1
                self.total_pods_carried += 1
            self.state = "idle"
            self.current_task = None
    
    def assign_task(self, task, pod_position, all_robots):
        self.current_task = task
        if self.plan_path_to(pod_position, all_robots):
            self.state = "moving_to_pod"
            return True
        return False
    
    def pickup_pod(self, pod_id):
        if self.carrying_pod:
            return False
        
        pod_pos = self.world.pod_locations.get(pod_id)
        if pod_pos != self.pos:
            return False
        
        if pod_id in self.world.pod_locations:
            del self.world.pod_locations[pod_id]
        
        self.carrying_pod = StoragePod(pod_id, self.x, self.y)
        self.carrying_pod.is_carried = True
        self.carrying_pod.carried_by = self.id
        self.carrying_pod.color = COLORS['POD_CARRIED']
        
        self.color = COLORS['ROBOT_BUSY']
        return True
    
    def drop_pod(self):
        if not self.carrying_pod:
            return False
        
        self.world.pod_locations[self.carrying_pod.id] = self.pos
        
        self.carrying_pod.is_carried = False
        self.carrying_pod.carried_by = None
        self.carrying_pod.color = COLORS['POD_STORED']
        
        self.carrying_pod = None
        self.color = COLORS['ROBOT_IDLE']
        return True
    
    def update(self, all_robots):
        self.update_priority()
        
        if self.battery <= 0:
            self.battery = 0
            self.state = "dead"
            self.color = COLORS['DARK_GREY']
            return
        
        if (self.battery < BATTERY_THRESHOLD and 
            self.state not in ["charging", "dead"] and
            not self.carrying_pod):
            
            nearest_charging = min(self.world.charging_stations,
                                 key=lambda pos: manhattan_distance(self.pos, pos))
            
            if self.plan_path_to(nearest_charging, all_robots):
                self.state = "charging"
                self.current_task = None
        
        if self.state == "charging":
            self.battery = min(MAX_BATTERY, self.battery + 3)
            self.color = COLORS['ROBOT_CHARGING']
            
            if self.battery >= MAX_BATTERY * 0.95:
                self.state = "idle"
                self.color = COLORS['ROBOT_IDLE']
        
        self.tick_counter += 1
        if self.tick_counter >= self.speed and self.path:
            if self.move_step(all_robots):
                self.tick_counter = 0
        
        if self.current_task and self.state not in ["charging", "dead"]:
            self.execute_task_sequence(all_robots)

class CentralDispatcher:
    def __init__(self, world, robots, pods_dict):
        self.world = world
        self.robots = robots
        self.pods_dict = pods_dict
        
        self.task_queue = deque()
        self.completed_tasks = 0
        self.failed_tasks = 0
        self.task_id_counter = 1
        
        self.performance_stats = {
            'total_orders': 0,
            'avg_completion_time': 0,
            'robot_utilization': 0,
            'total_collisions': 0
        }
    
    def create_order(self, pod_id, pick_station_id):
        if pod_id not in self.pods_dict:
            return None
        
        pod = self.pods_dict[pod_id]
        if not pod.items:
            return None
        
        items_to_pick = random.sample(pod.items, min(3, len(pod.items)))
        
        task = Task(
            id=self.task_id_counter,
            pod_id=pod_id,
            pick_station_id=pick_station_id,
            status="pending",
            items=items_to_pick,
            assigned_robot=None
        )
        
        self.task_queue.append(task)
        self.task_id_counter += 1
        self.performance_stats['total_orders'] += 1
        
        return task
    
    def assign_tasks(self):
        if not self.task_queue:
            return
        
        available_robots = [
            r for r in self.robots 
            if r.state in ["idle", "charging"] and 
            r.battery > BATTERY_THRESHOLD + 10 and
            not r.current_task
        ]
        
        if not available_robots:
            return
        
        tasks_to_remove = []
        
        for task in self.task_queue:
            if task.pod_id not in self.world.pod_locations:
                tasks_to_remove.append(task)
                continue
            
            pod_position = self.world.pod_locations[task.pod_id]
            pick_station_pos = self.world.get_pick_station_position(task.pick_station_id)
            
            if not pod_position or not pick_station_pos:
                continue
            
            best_robot = None
            best_score = float('inf')
            
            for robot in available_robots:
                distance = manhattan_distance(robot.pos, pod_position)
                battery_score = (MAX_BATTERY - robot.battery) / 10
                priority_score = robot.priority
                
                score = distance + battery_score - priority_score
                
                if score < best_score:
                    best_score = score
                    best_robot = robot
            
            if best_robot:
                if best_robot.assign_task(task, pod_position, self.robots):
                    task = task._replace(
                        status="assigned",
                        assigned_robot=best_robot.id
                    )
                    tasks_to_remove.append(task)
                    available_robots.remove(best_robot)
        
        for task in tasks_to_remove:
            if task in self.task_queue:
                self.task_queue.remove(task)
    
    def generate_random_orders(self, probability=0.02):
        if (random.random() < probability and 
            self.world.pod_locations and 
            len(self.task_queue) < 15):
            
            pod_id = random.choice(list(self.world.pod_locations.keys()))
            station_id = random.randint(0, NUM_PICK_STATIONS - 1)
            self.create_order(pod_id, station_id)
    
    def update_statistics(self):
        active_robots = sum(1 for r in self.robots if r.state not in ["charging", "dead", "idle"])
        total_robots = len(self.robots)
        total_collisions = sum(r.collision_count for r in self.robots)
        
        if total_robots > 0:
            self.performance_stats['robot_utilization'] = active_robots / total_robots * 100
            self.performance_stats['total_collisions'] = total_collisions

class WarehouseVisualization:
    def __init__(self, screen):
        self.screen = screen
        self.font_small = pygame.font.SysFont("Arial", 12)
        self.font_medium = pygame.font.SysFont("Arial", 14)
        self.font_large = pygame.font.SysFont("Arial", 18, bold=True)
    
    def draw_grid(self, world):
        for x in range(world.width):
            for y in range(world.height):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, COLORS['GRID_LINE'], rect, 1)
    
    def draw_obstacles(self, world):
        for obstacle in world.obstacles:
            x, y = obstacle
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(self.screen, COLORS['OBSTACLE'], rect)
    
    def draw_pick_stations(self, world):
        for x, y, station_id in world.pick_stations:
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(self.screen, COLORS['PICK_STATION'], rect)
            
            station_text = self.font_medium.render(f"P{station_id}", True, COLORS['WHITE'])
            text_rect = station_text.get_rect(center=rect.center)
            self.screen.blit(station_text, text_rect)
    
    def draw_charging_stations(self, world):
        for x, y in world.charging_stations:
            rect = pygame.Rect(x * CELL_SIZE + 4, y * CELL_SIZE + 4, CELL_SIZE - 8, CELL_SIZE - 8)
            pygame.draw.rect(self.screen, COLORS['CHARGING_STATION'], rect)
            
            charge_text = self.font_medium.render("⚡", True, COLORS['BLACK'])
            text_rect = charge_text.get_rect(center=rect.center)
            self.screen.blit(charge_text, text_rect)
    
    def draw_pods(self, pods_dict, world):
        for pod in pods_dict.values():
            if not pod.is_carried:
                x, y = pod.x, pod.y
                rect = pygame.Rect(x * CELL_SIZE + 2, y * CELL_SIZE + 2, CELL_SIZE - 4, CELL_SIZE - 4)
                pygame.draw.rect(self.screen, pod.color, rect)
                
                pod_text = self.font_small.render(str(pod.id), True, COLORS['WHITE'])
                text_rect = pod_text.get_rect(center=rect.center)
                self.screen.blit(pod_text, text_rect)
    
    def draw_robots(self, robots):
        for robot in robots:
            x, y = robot.x, robot.y
            center_x = x * CELL_SIZE + CELL_SIZE // 2
            center_y = y * CELL_SIZE + CELL_SIZE // 2
            radius = CELL_SIZE // 2 - 2
            
            pygame.draw.circle(self.screen, robot.color, (center_x, center_y), radius)
            pygame.draw.circle(self.screen, COLORS['BLACK'], (center_x, center_y), radius, 1)
            
            id_text = self.font_small.render(str(robot.id), True, COLORS['WHITE'])
            id_rect = id_text.get_rect(center=(center_x, center_y))
            self.screen.blit(id_text, id_rect)
            
            if robot.path:
                for i in range(len(robot.path) - 1):
                    start_pos = robot.path[i]
                    end_pos = robot.path[i + 1]
                    
                    start_x = start_pos[0] * CELL_SIZE + CELL_SIZE // 2
                    start_y = start_pos[1] * CELL_SIZE + CELL_SIZE // 2
                    end_x = end_pos[0] * CELL_SIZE + CELL_SIZE // 2
                    end_y = end_pos[1] * CELL_SIZE + CELL_SIZE // 2
                    
                    pygame.draw.line(self.screen, COLORS['PATH'], (start_x, start_y), (end_x, end_y), 2)
            
            battery_width = CELL_SIZE - 6
            battery_height = 3
            battery_x = x * CELL_SIZE + 3
            battery_y = y * CELL_SIZE + CELL_SIZE - 6
            
            charge_level = (robot.battery / MAX_BATTERY) * battery_width
            
            pygame.draw.rect(self.screen, COLORS['BLACK'], 
                           (battery_x, battery_y, battery_width, battery_height), 1)
            
            if robot.battery > BATTERY_THRESHOLD:
                battery_color = COLORS['SUCCESS']
            else:
                battery_color = COLORS['ERROR']
                
            pygame.draw.rect(self.screen, battery_color,
                           (battery_x, battery_y, charge_level, battery_height))
            
            if robot.carrying_pod:
                pod_indicator = pygame.Rect(center_x - 6, center_y - 10, 12, 6)
                pygame.draw.rect(self.screen, COLORS['POD_CARRIED'], pod_indicator)
    
    def draw_status_panel(self, dispatcher, robots, world):
        panel_y = GRID_HEIGHT * CELL_SIZE
        panel_height = SCREEN_HEIGHT - panel_y
        
        panel_rect = pygame.Rect(0, panel_y, SCREEN_WIDTH, panel_height)
        pygame.draw.rect(self.screen, COLORS['LIGHT_GREY'], panel_rect)
        pygame.draw.line(self.screen, COLORS['DARK_GREY'], (0, panel_y), (SCREEN_WIDTH, panel_y), 2)
        
        active_robots = sum(1 for r in robots if r.state not in ["charging", "dead", "idle"])
        charging_robots = sum(1 for r in robots if r.state == "charging")
        busy_robots = sum(1 for r in robots if r.carrying_pod is not None)
        dead_robots = sum(1 for r in robots if r.state == "dead")
        total_collisions = dispatcher.performance_stats['total_collisions']
        
        stats = [
            f"KIVA WAREHOUSE SYSTEM - ROBOT COORDINATION",
            f"Робот: {active_robots}/{len(robots)} ажиллаж байна "
            f"({charging_robots} цэнэглэж, {busy_robots} ачаатай, {dead_robots} dead)",
            f"Мөргөлдөл: {total_collisions} удаа",
            f"Даалгавар: {len(dispatcher.task_queue)} хүлээгдэж байна "
            f"({dispatcher.completed_tasks} дууссан)",
            f"Pod: {len(world.pod_locations)}/{NUM_PODS} байршилд",
            f"Системийн цаг: {world.current_time}",
            f"Роботын ашиглалт: {dispatcher.performance_stats['robot_utilization']:.1f}%"
        ]
        
        for i, stat in enumerate(stats):
            text = self.font_medium.render(stat, True, COLORS['TEXT'])
            self.screen.blit(text, (10, panel_y + 10 + i * 25))
        
        detail_start_y = panel_y + 160
        detail_title = self.font_large.render("Роботын Статус:", True, COLORS['TEXT'])
        self.screen.blit(detail_title, (10, detail_start_y))
        
        for i, robot in enumerate(robots[:8]):
            status_color = COLORS['SUCCESS'] if robot.collision_count == 0 else COLORS['WARNING']
            status_info = f"Робот {robot.id}: {robot.state} | Батарей: {robot.battery:.1f}% | Мөргөлдөл: {robot.collision_count}"
            if robot.current_task:
                status_info += f" | Даалгавар: {robot.current_task.id}"
            
            status_text = self.font_small.render(status_info, True, status_color)
            self.screen.blit(status_text, (10, detail_start_y + 30 + i * 20))
        
        controls_y = detail_start_y + 200
        controls = [
            "Удирдлага: SPACE - Зогсоох/Үргэжлүүлэх, T - Шинэ захиалга, R - Цэнэглэх, C - Статистик"
        ]
        
        for i, control in enumerate(controls):
            control_text = self.font_small.render(control, True, COLORS['DARK_GREY'])
            self.screen.blit(control_text, (10, controls_y + i * 20))
    
    def draw_all(self, world, pods_dict, robots, dispatcher):
        self.screen.fill(COLORS['WHITE'])
        
        self.draw_grid(world)
        self.draw_obstacles(world)
        self.draw_pick_stations(world)
        self.draw_charging_stations(world)
        self.draw_pods(pods_dict, world)
        self.draw_robots(robots)
        self.draw_status_panel(dispatcher, robots, world)
        
        pygame.display.flip()

class KivaWarehouseSimulation:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Amazon Kiva Robot System - Advanced Collision Avoidance")
        self.clock = pygame.time.Clock()
        
        self.world = WarehouseGrid()
        self.robots = []
        self.pods_dict = {}
        self.dispatcher = None
        self.visualization = WarehouseVisualization(self.screen)
        
        self.running = True
        self.paused = False
        self.show_stats = False
        
        self.initialize_simulation()
    
    def initialize_simulation(self):
        print("Kiva Warehouse System эхлүүлж байна...")
        
        for pod_id, pos in self.world.pod_locations.items():
            self.pods_dict[pod_id] = StoragePod(pod_id, pos[0], pos[1])
        
        for i in range(NUM_ROBOTS):
            start_pos = self.world.charging_stations[i % len(self.world.charging_stations)]
            robot = KivaRobot(i + 1, start_pos[0], start_pos[1], self.world)
            self.robots.append(robot)
            self.world.reserve_cell(start_pos, robot.id)
        
        self.dispatcher = CentralDispatcher(self.world, self.robots, self.pods_dict)
        
        for _ in range(6):
            if self.world.pod_locations:
                pod_id = random.choice(list(self.world.pod_locations.keys()))
                station_id = random.randint(0, NUM_PICK_STATIONS - 1)
                self.dispatcher.create_order(pod_id, station_id)
        
        print(f"Симуляци эхлэв: {NUM_ROBOTS} робот, {len(self.pods_dict)} pod")
    
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_t and not self.paused:
                    if self.world.pod_locations:
                        pod_id = random.choice(list(self.world.pod_locations.keys()))
                        station_id = random.randint(0, NUM_PICK_STATIONS - 1)
                        self.dispatcher.create_order(pod_id, station_id)
                elif event.key == pygame.K_r and not self.paused:
                    for robot in self.robots:
                        if robot.state != "dead":
                            robot.battery = min(MAX_BATTERY, robot.battery + 30)
                elif event.key == pygame.K_c:
                    self.show_statistics()
    
    def show_statistics(self):
        total_distance = sum(r.distance_traveled for r in self.robots)
        total_tasks = sum(r.tasks_completed for r in self.robots)
        total_collisions = sum(r.collision_count for r in self.robots)
        
        print("\n" + "="*50)
        print("KIVA СИСТЕМИЙН СТАТИСТИК")
        print("="*50)
        print(f"Нийт робот: {len(self.robots)}")
        print(f"Нийт мөргөлдөл: {total_collisions}")
        print(f"Нийт явсан зай: {total_distance} нүд")
        print(f"Нийт гүйцэтгэсэн даалгавар: {total_tasks}")
        print(f"Системийн ажилласан цаг: {self.world.current_time}")
        
        for robot in self.robots:
            print(f"  Робот {robot.id}: {robot.tasks_completed} даалгавар, "
                  f"{robot.collision_count} мөргөлдөл, {robot.battery:.1f}% батарей")
        print("="*50)
    
    def update_simulation(self):
        if self.paused:
            return
        
        self.world.update_time()
        self.dispatcher.generate_random_orders(0.02)
        self.dispatcher.assign_tasks()
        
        for robot in self.robots:
            robot.update(self.robots)
        
        self.dispatcher.update_statistics()
    
    def run(self):
        print("Симуляци ажиллаж байна...")
        print("Товчлуурууд: SPACE-зогсоох, T-шинэ захиалга, R-цэнэглэх, C-статистик")
        
        while self.running:
            self.handle_events()
            self.update_simulation()
            self.visualization.draw_all(self.world, self.pods_dict, self.robots, self.dispatcher)
            self.clock.tick(10)
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    simulation = KivaWarehouseSimulation()
    simulation.run()