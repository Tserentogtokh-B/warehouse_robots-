# warehouse_sim.py
"""
Олон-агент Kiva-style warehouse симуляц (Python + Pygame).
Санал болгож буй функцууд:
- Grid дээр A* pathfinding
- Central Dispatcher (task allocation: nearest idle robot)
- Robots: move, reserve next cell, pickup/putdown pod, go to charge
- Simple collision avoidance via cell reservation
- Pygame visualization
Бүх комментууд монгол хэл дээр байна.
"""

import pygame
import sys
import math
import heapq
import random
from collections import deque, defaultdict, namedtuple

# ---------- Тохиргоо ----------
CELL_SIZE = 28           # тор нүдийн пиксел хэмжээ
GRID_W = 24              # тор өргөн (нүд)
GRID_H = 18              # тор өндөр (нүд)
SCREEN_W = CELL_SIZE * GRID_W
SCREEN_H = CELL_SIZE * GRID_H + 60  # дооро хэсэг status-т зориулж
FPS = 30

NUM_ROBOTS = 6
NUM_PODS = 40
NUM_PICK_STATIONS = 3
CHARGING_STATIONS = [(0, GRID_H-1), (1, GRID_H-1)]

ROBOT_SPEED = 4  # нүд тутам хэдийн хөдлөх кадр тутамд биш (movement ticks per cell)
BATTERY_CAPACITY = 200  # энгийн батарей хэмжүүр
BATTERY_THRESHOLD = 40  # доогуур бол цэнэглэх шаардлагатай

# өнгө
WHITE = (245,245,245)
BLACK = (20,20,20)
LIGHT_GREY = (200,200,200)
POD_COLOR = (200,120,50)
ROBOT_COLOR = (50,130,200)
PICK_COLOR = (120,200,120)
CHARGE_COLOR = (220,200,60)
TEXT_COLOR = (10,10,10)
RESERVED_CELL = (180,180,230)

# ---------- Төрөлүүд ----------
Task = namedtuple("Task", ["id", "pod_id", "pick_station", "status"])  # status: "queued","in_progress","done"

# ---------- A* Pathfinding ----------
def heuristic(a, b):
    # Manhattan metric (grid)
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def neighbors(cell):
    x,y = cell
    for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx,ny = x+dx, y+dy
        if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
            yield (nx,ny)

def astar(start, goal, blocked=set()):
    # blocked: set of cells that cannot be traversed (e.g., obstacles)
    if start == goal:
        return [start]
    open_heap = []
    heapq.heappush(open_heap, (0 + heuristic(start,goal), 0, start, None))
    came_from = {}
    gscore = {start: 0}
    while open_heap:
        f, g, current, _ = heapq.heappop(open_heap)
        if current == goal:
            # reconstruct
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node, None)
            return path[::-1]
        for nb in neighbors(current):
            if nb in blocked:
                continue
            tentative_g = g + 1
            if tentative_g < gscore.get(nb, 1e9):
                gscore[nb] = tentative_g
                priority = tentative_g + heuristic(nb, goal)
                heapq.heappush(open_heap, (priority, tentative_g, nb, current))
                came_from[nb] = current
    return None  # зам байхгүй

# ---------- Grid, Pods, Stations ----------
class GridWorld:
    def __init__(self):
        self.width = GRID_W
        self.height = GRID_H
        self.pods = {}  # pod_id -> (x,y)
        self.pick_stations = []  # list of (x,y)
        self.charging = CHARGING_STATIONS.copy()
        self.obstacles = set()  # боломжит саад
        # cell reservation: (x,y) -> robot_id who reserved
        self.reservations = {}

    def is_free(self, cell):
        # free if not obstacle and not reserved
        return cell not in self.obstacles and cell not in self.reservations

    def reserve(self, cell, robot_id):
        # захиалга хийх (энгийн): нэг нүдийг нэг робот л захиалан авах
        if cell in self.reservations and self.reservations[cell] != robot_id:
            return False
        self.reservations[cell] = robot_id
        return True

    def release(self, cell, robot_id):
        owner = self.reservations.get(cell)
        if owner == robot_id:
            del self.reservations[cell]

    def add_pod(self, pod_id, pos):
        self.pods[pod_id] = pos

    def remove_pod(self, pod_id):
        if pod_id in self.pods:
            del self.pods[pod_id]

# ---------- Robot ----------
class Robot:
    def __init__(self, robot_id, start_pos, world: GridWorld):
        self.id = robot_id
        self.pos = start_pos  # current cell (x,y)
        self.world = world
        self.carrying = None  # pod_id if carrying
        self.path = deque()  # planned path of cells
        self.state = "idle"  # idle, to_pod, to_pick, to_storage, charging, moving
        self.tick_counter = 0
        self.speed = ROBOT_SPEED
        self.battery = BATTERY_CAPACITY
        self.goal = None
        self.reserving = set()  # set of reserved future cells
        self.current_task = None

    def plan_path(self, goal):
        # compute path avoiding obstacles; include reservations of other robots as blocked
        blocked = set(self.world.obstacles)
        # exclude cells reserved by this robot
        blocked |= set([c for c,owner in self.world.reservations.items() if owner != self.id])
        path = astar(self.pos, goal, blocked=blocked)
        if path is None:
            return False
        # store path excluding current position
        self.path = deque(path[1:])
        self.goal = goal
        return True

    def step(self):
        # power drain
        if self.state != "charging":
            self.battery -= 0.02  # idle drain
        # if need charging urgently, override tasks
        if self.battery <= 0:
            self.battery = 0
            self.state = "dead"
            self.path.clear()
            return

        # execute movement ticks (simple speed control)
        self.tick_counter += 1
        if self.tick_counter < self.speed:
            return
        self.tick_counter = 0

        # movement
        if self.path:
            next_cell = self.path[0]
            # try reserve next cell
            ok = self.world.reserve(next_cell, self.id)
            if not ok:
                # can't move, wait and maybe replan later
                return
            # release current pos reservation held by self (if any)
            self.world.release(self.pos, self.id)
            # move to next cell
            self.pos = next_cell
            # pop path head
            self.path.popleft()
            # battery consume
            self.battery -= 0.5
            return

        # if no path, decide based on state
        if self.state == "to_pod":
            # we've arrived at pod (or failed to plan)
            self.state = "at_pod"
        elif self.state == "to_pick":
            self.state = "at_pick"
        elif self.state == "to_storage":
            self.state = "idle"
            self.current_task = None
        elif self.state == "charging":
            # charge a bit
            self.battery += 3
            if self.battery >= BATTERY_CAPACITY:
                self.battery = BATTERY_CAPACITY
                self.state = "idle"
        # else idle -> nothing

    def assign_task(self, task, pod_pos, pick_pos, storage_pos):
        # assign and plan path sequence: go to pod -> pick station -> storage
        self.current_task = task
        # 1) go to pod
        self.state = "to_pod"
        if not self.plan_path(pod_pos):
            # can't plan; fail
            self.state = "idle"
            self.current_task = None
            return False
        # reserve current pos as occupied
        self.world.reserve(self.pos, self.id)
        return True

    def pickup_pod(self, pod_id):
        # called when at pod cell
        if self.carrying is None:
            self.carrying = pod_id
            # remove pod from world storage (it's now on the robot)
            if pod_id in self.world.pods:
                del self.world.pods[pod_id]
            return True
        return False

    def drop_pod_at(self, pos):
        if self.carrying is not None:
            self.world.pods[self.carrying] = pos
            self.carrying = None
            return True
        return False

# ---------- Dispatcher (Fleet Manager) ----------
class Dispatcher:
    def __init__(self, world: GridWorld, robots: list):
        self.world = world
        self.robots = robots
        self.task_queue = deque()
        self.task_counter = 0
        self.pod_storage_positions = list(world.pods.items())  # list of (pod_id,pos)

    def create_task(self, pod_id, pick_station):
        self.task_counter += 1
        t = Task(id=self.task_counter, pod_id=pod_id, pick_station=pick_station, status="queued")
        self.task_queue.append(t)

    def assign_tasks(self):
        # simple nearest-robot allocation for queued tasks
        if not self.task_queue:
            return
        for _ in range(len(self.task_queue)):
            task = self.task_queue[0]
            # find pod location
            pod_pos = self.world.pods.get(task.pod_id)
            if pod_pos is None:
                # pod missing (maybe being carried) -> skip for now
                self.task_queue.rotate(-1)
                continue
            # find nearest available robot (idle and battery ok)
            candidates = []
            for r in self.robots:
                if r.state in ("idle",) and r.current_task is None and r.battery > BATTERY_THRESHOLD:
                    dist = heuristic(r.pos, pod_pos)
                    candidates.append((dist, r))
            if not candidates:
                # no available robot now
                self.task_queue.rotate(-1)
                continue
            candidates.sort(key=lambda x: x[0])
            robot = candidates[0][1]
            pick_pos = self.world.pick_stations[task.pick_station]
            storage_pos = pod_pos  # for simplicity return to same slot
            ok = robot.assign_task(task, pod_pos, pick_pos, storage_pos)
            if ok:
                task = task._replace(status="in_progress")
                self.task_queue.popleft()
                # set robot follow-up: after reaching pod, robot should plan to pick->pick station->return
                # To keep simple, we'll set robot's path to pod; further planning will be done when robot reaches pod.
            else:
                self.task_queue.rotate(-1)

    def generate_random_tasks(self, prob=0.01):
        # randomly create tasks
        if random.random() < prob and self.world.pods:
            pod_id = random.choice(list(self.world.pods.keys()))
            pick_idx = random.randrange(len(self.world.pick_stations))
            self.create_task(pod_id, pick_idx)

# ---------- Simulation Manager ----------
class Simulation:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("Kiva-like Multi-Agent Warehouse Simulation")
        self.clock = pygame.time.Clock()
        self.world = GridWorld()
        self.robots = []
        self.dispatcher = None
        self.font = pygame.font.SysFont("Arial", 14)
        self.init_world()
        self.running = True
        self.paused = False

    def init_world(self):
        # place pick stations at top row
        for i in range(NUM_PICK_STATIONS):
            x = GRID_W - 1 - i*3
            y = 0
            self.world.pick_stations.append((x,y))

        # place pods in storage area (clustered)
        pod_id = 1
        placed = 0
        for y in range(4, GRID_H-4):
            for x in range(2, GRID_W-4):
                if placed >= NUM_PODS:
                    break
                # skip pick and charging cells
                if (x,y) in self.world.pick_stations or (x,y) in self.world.charging:
                    continue
                self.world.add_pod(pod_id, (x,y))
                pod_id += 1
                placed += 1
            if placed >= NUM_PODS:
                break

        # add some random obstacles for demonstration
        for _ in range(12):
            ox = random.randrange(3, GRID_W-3)
            oy = random.randrange(3, GRID_H-3)
            self.world.obstacles.add((ox,oy))

        # create robots at charging stations initially
        for i in range(NUM_ROBOTS):
            pos = self.world.charging[i % len(self.world.charging)]
            r = Robot(i+1, pos, self.world)
            # reserve their start cell
            self.world.reserve(pos, r.id)
            r.state = "charging"
            self.robots.append(r)

        # Dispatcher
        self.dispatcher = Dispatcher(self.world, self.robots)

        # Pre-generate some tasks
        pod_ids = list(self.world.pods.keys())
        for _ in range(6):
            if not pod_ids:
                break
            self.dispatcher.create_task(random.choice(pod_ids), random.randrange(len(self.world.pick_stations)))

    def update(self):
        # dispatcher may generate random tasks
        self.dispatcher.generate_random_tasks(prob=0.03)
        # assign tasks
        self.dispatcher.assign_tasks()

        # step robots
        for r in self.robots:
            # react to reaching pod or pick station
            if r.current_task:
                task = r.current_task
                # if robot state is 'at_pod' and carries nothing -> pick up
                if r.state == "at_pod":
                    # at pod cell?
                    pod_pos = self.world.pods.get(task.pod_id)
                    if pod_pos is None and r.carrying != task.pod_id:
                        # maybe another robot took it; abort
                        r.state = "idle"
                        r.current_task = None
                    else:
                        # pickup
                        r.pickup_pod(task.pod_id)
                        # now plan to pick station
                        pick_pos = self.world.pick_stations[task.pick_station]
                        r.state = "to_pick"
                        r.plan_path(pick_pos)
                elif r.state == "at_pick":
                    # simulate human pick (instant)
                    # drop pod at pick station (robot holds pod while picking; after picking, we return it)
                    if r.carrying is not None:
                        # drop at station for pick (or keep with robot, choose to keep)
                        # For realism, leave pod at station then return it
                        station_pos = self.world.pick_stations[task.pick_station]
                        r.drop_pod_at(station_pos)
                    # mark task done and plan to return pod to storage
                    r.current_task = None
                    r.state = "to_storage"
                    # choose a nearby empty storage spot (simple: original pod pos if free else random free)
                    original = r.world.pods.get(task.pod_id)
                    # find empty storage
                    # For simplicity: return to a random storage near center
                    storage_pos = self.find_free_storage()
                    if storage_pos:
                        r.plan_path(storage_pos)
                elif r.state == "to_storage":
                    # arrived -> if carrying none (we left at pick station), just put pod back to storage
                    # For simplicity, if pod is not being carried, create a new pod id to represent returned inventory
                    if task.pod_id not in r.world.pods:
                        # return a pod object
                        r.world.pods[task.pod_id] = r.pos
                    r.state = "idle"
                    r.current_task = None

            # autonomous decide to go charge
            if r.battery < BATTERY_THRESHOLD and r.state not in ("charging", "to_pod", "to_pick"):
                # find nearest charging station
                cs = min(self.world.charging, key=lambda c: heuristic(r.pos, c))
                r.state = "to_charge"
                r.plan_path(cs)

            # if reached charging cell and state to_charge -> set charging
            if r.state == "to_charge" and r.pos in self.world.charging:
                r.state = "charging"
                r.path.clear()

            # robot step
            r.step()

        # cleanup reservations for robots that have left cells (release stale reservations owned by dead robots)
        # (already handled by Robot.step releasing previous position)
        # ensure charging robots reserve their pos
        for r in self.robots:
            if r.state == "charging" or r.path:
                self.world.reserve(r.pos, r.id)

    def find_free_storage(self):
        # find a free cell in storage area not occupied by pods or obstacles or robots
        for y in range(4, GRID_H-4):
            for x in range(2, GRID_W-4):
                cell = (x,y)
                if cell in self.world.obstacles:
                    continue
                if cell in self.world.pods:
                    continue
                # not reserved by other robots
                if cell in self.world.reservations:
                    continue
                return cell
        return None

    def draw(self):
        self.screen.fill(WHITE)
        # draw grid
        for x in range(GRID_W):
            for y in range(GRID_H):
                rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, LIGHT_GREY, rect, 1)
                # reserved cell shading
                if (x,y) in self.world.reservations:
                    pygame.draw.rect(self.screen, RESERVED_CELL, rect)

        # draw obstacles
        for (ox,oy) in self.world.obstacles:
            r = pygame.Rect(ox*CELL_SIZE, oy*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(self.screen, BLACK, r)

        # draw pods
        for pid, pos in self.world.pods.items():
            x,y = pos
            r = pygame.Rect(x*CELL_SIZE+4, y*CELL_SIZE+4, CELL_SIZE-8, CELL_SIZE-8)
            pygame.draw.rect(self.screen, POD_COLOR, r)
            # pod id small text
            txt = self.font.render(str(pid), True, BLACK)
            self.screen.blit(txt, (x*CELL_SIZE+2, y*CELL_SIZE+2))

        # draw pick stations
        for i,ps in enumerate(self.world.pick_stations):
            x,y = ps
            r = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(self.screen, PICK_COLOR, r)
            txt = self.font.render(f"P{i}", True, BLACK)
            self.screen.blit(txt, (x*CELL_SIZE+2, y*CELL_SIZE+2))

        # draw charging
        for cs in self.world.charging:
            x,y = cs
            r = pygame.Rect(x*CELL_SIZE+2, y*CELL_SIZE+2, CELL_SIZE-4, CELL_SIZE-4)
            pygame.draw.rect(self.screen, CHARGE_COLOR, r)
            txt = self.font.render("C", True, BLACK)
            self.screen.blit(txt, (x*CELL_SIZE+6, y*CELL_SIZE+4))

        # draw robots
        for rbt in self.robots:
            x,y = rbt.pos
            cx = x*CELL_SIZE + CELL_SIZE//2
            cy = y*CELL_SIZE + CELL_SIZE//2
            radius = CELL_SIZE//2 - 4
            pygame.draw.circle(self.screen, ROBOT_COLOR, (cx,cy), radius)
            # battery bar
            bx = x*CELL_SIZE + 2
            by = y*CELL_SIZE + CELL_SIZE - 6
            bw = int((CELL_SIZE-4) * (rbt.battery / BATTERY_CAPACITY))
            pygame.draw.rect(self.screen, BLACK, (bx,by,CELL_SIZE-4,4), 1)
            pygame.draw.rect(self.screen, (50,200,50), (bx,by,bw,4))
            # id text
            idtxt = self.font.render(str(rbt.id), True, BLACK)
            self.screen.blit(idtxt, (x*CELL_SIZE+4,y*CELL_SIZE+2))
            # if carrying, display small pod rect on top
            if rbt.carrying:
                wx = x*CELL_SIZE + CELL_SIZE//2
                wy = y*CELL_SIZE + 4
                pygame.draw.rect(self.screen, POD_COLOR, (wx-6, wy-6, 12, 8))

        # bottom status area
        pygame.draw.rect(self.screen, LIGHT_GREY, (0, GRID_H*CELL_SIZE, SCREEN_W, 60))
        # text summaries
        lines = [
            f"Tasks queued: {len(self.dispatcher.task_queue)}",
            f"Pods stored: {len(self.world.pods)}",
            f"Robots: {len(self.robots)} (Battery threshold: {BATTERY_THRESHOLD})",
            f"Press SPACE to pause/resume, T to spawn task"
        ]
        for i,ln in enumerate(lines):
            txt = self.font.render(ln, True, TEXT_COLOR)
            self.screen.blit(txt, (8, GRID_H*CELL_SIZE + 6 + i*14))

        pygame.display.flip()

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_t:
                        # spawn an on-demand task
                        if self.world.pods:
                            pid = random.choice(list(self.world.pods.keys()))
                            pidx = random.randrange(len(self.world.pick_stations))
                            self.dispatcher.create_task(pid, pidx)

            if not self.paused:
                self.update()
            self.draw()
            self.clock.tick(FPS)

        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()
