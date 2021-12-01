from collections import namedtuple
import heapq
from shapely.geometry import Polygon
from tkinter import *
import math


'''================= Your classes and methods ================='''

# These functions will help you to check collisions with obstacles
position = namedtuple('position', 'x y angle')

def position2poygon(position, w, h) :
    x, y, angle = position
    points = [(x - w/2, y - h/2), (x + w/2, y - h/2), (x + w/2, y + h/2), (x - w/2, y + h/2)] 
    new_points = rotation(points, angle, (x, y))
    return Polygon(new_points)

def obstackle2polygon(obstacle) :
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])] 
    return Polygon(points)

def rotation(points, angle, center):
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_prev, y_prev in points:
        x_prev -= cx
        y_prev -= cy
        x_new = x_prev * cos_val - y_prev * sin_val
        y_new = x_prev * sin_val + y_prev * cos_val
        new_points.append((x_new + cx, y_new + cy))

    return new_points



def collides(position, obstacle) :
    return position2poygon(position, w=1.5*100, h=1.5*200).intersects(obstackle2polygon(obstacle))
        


def step(state, dist, delta):
    next_yaw = state.angle + delta
    next_x = state.x + dist * math.sin(next_yaw)
    next_y = state.y - dist * math.cos(next_yaw)
    return position(next_x, next_y, next_yaw)

class Path:
    def __init__(self, x_max, y_max, start_pos, target_pos, obstacles, dist=50, delta=10):
        self.x_max = x_max
        self.y_max = y_max
        self.start_state =  position(*start_pos)
        self.target_state = position(*target_pos)
        self.obstacles = obstacles
        self.dist = dist
        self.delta = delta
    
    def is_target_reachable(self, state):
        distance, angle = self.angle_to_target(state)
        out = ((state.angle - angle)**2 < 1 * self.delta and
            (self.target_state.angle - angle)**2 < 1 * self.delta and
               distance < 1 * self.dist
              )
        return out
    def angle_to_target(self, state):
        dx = self.target_state.x - state.x
        dy = self.target_state.y - state.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle = math.atan2(dx, -dy)
        return distance, angle


    def get_successors(self, state):
        actions = [
            (self.dist, self.delta),
            (self.dist, -self.delta),
            (self.dist,  0),
           
        ]
        next_states = [
            step(state, dist, delta)
            for dist, delta in actions
        ]
        potential_nodes = [
            state for state in next_states
            if self.is_path_availible(state)
        ]
        return potential_nodes
    
    def is_path_availible(self, state):
        if state.x < 0 or state.y < 0 or state.x > self.x_max or state.y > self.y_max:
            return False

        collisions = [
            collides((state.x, state.y, state.angle), obstacle)
            for obstacle in self.obstacles
        ]
        return sum(collisions) == 0

    def h(self, state):
        distance, angle = self.angle_to_target(state)
        return abs(distance - self.dist) + (self.target_state.angle - angle)**2 


    def reconstruct_path(self, node):
        path = [self.target_state]
        while node:
            path.append(node.state)
            node = node.parent
        path = path[::-1]
        
        return path


    def path_search(self):
        Node = namedtuple('Node', 'state parent step')
        root_node = Node(self.start_state, None, 0)
        history = [self.start_state]
        frontier = []
        heapq.heappush(frontier, (0,root_node))
        find_path = None
        while frontier:
            _, node = heapq.heappop(frontier)
            if len(history) > 100000:
                break
            if self.is_target_reachable(node.state):
                find_path = self.reconstruct_path(node)
                break
            for next_state in self.get_successors(node.state):
                history.append(next_state)
                priority = node.step + 1 + self.h(next_state)
                next_node = Node(next_state, node, node.step + 1)
                heapq.heappush(frontier, (priority, next_node))
        return find_path, history


    

class Window():
        
    '''================= Your Main Function ================='''
    
    def go(self, event):
        
        
        path = Path( self.width, 
                                        self.height,
                                        self.get_start_position(),
                                        self.get_target_position(),
                                        self.get_obstacles(),
                                        dist = 150,
                                        delta = math.radians(60),
                                        )
        path, history =path.path_search()
        print('path=', path)
        self.plot_path(history, outline="black", w=10, h=20)
        if path:
            self.plot_path(path, outline="blue", w=25, h=50)
    
            

    def plot_pos(self, center_x, center_y, angle, w=30, h=80, outline="black"):

        block = [[center_x - w, center_y - h],
                 [center_x + w, center_y - h],
                 [center_x + w, center_y + h],
                 [center_x - w, center_y + h]]
        block = rotation(block, angle, (center_x, center_y))

        id = self.canvas.create_polygon(block, outline=outline, fill="")
        return id


    def plot_path(self, pos_list, w=30, h=80, outline="red"):
        for x, y, angle in pos_list:
            self.plot_pos(x, y, angle, w, h, outline)

        
    '''================= Interface Methods ================='''
    
    def get_obstacles(self) :
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2) :
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles
            
            
    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        angle = self.get_yaw(2)
        return x,y,angle
    
    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1 
        angle = self.get_yaw(1)
        return x,y,angle 
 

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y 
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw)
        else :
            return -math.acos(cos_yaw)
       
    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotation(self, points, angle, center):
        angle = math.angle_to_targetadians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_prev, y_prev in points:
            x_prev -= cx
            y_prev -= cy
            x_new = x_prev * cos_val - y_prev * sin_val
            y_new = x_prev * sin_val + y_prev * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.angle_to_targetotation([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.angle_to_targetoot

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.angle_to_targetoot = Tk()
        self.angle_to_targetoot.title("")
        self.width  = self.angle_to_targetoot.winfo_screenwidth()
        self.height = self.angle_to_targetoot.winfo_screenheight()
        self.angle_to_targetoot.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.angle_to_targetoot, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        # self.points = [0, 500, 500/2, 0, 500, 500]
    
if __name__ == "__main__":
    run = Window()
    run.run()
