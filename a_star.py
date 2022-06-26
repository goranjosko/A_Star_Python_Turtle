# I know the code is terribly messy, I’m a hobby programmer
import turtle
# import random
# import time as t


class Grid():

    def __init__(self, grid_size, node_size, start, goal):
        self.grid_size = grid_size
        self.node_size = node_size
        self.start = start
        self.goal = goal
        self.wall = []
        self.op_list = {}
        self.cl_list = {}

    def find_neighbors(self, node):
        # node positions around current node
        positions = [(-1, -1), (-1, 0), (-1, +1), (0, +1),
                     (+1, +1), (+1, 0), (+1, -1), (0, -1)]
        # take node coordinates and positions, spit out list of tuples
        neighbors = [tuple((x for x in map(sum, zip((node[0], node[1]), num))))
                     for num in positions]
        # remove nodes that are walls or outside of terrain (field)
        f_neighbors = [
            x for x in neighbors if (0 <= x[0] < self.grid_size and 0 <= x[1] < self.grid_size)
            and x not in self.wall]
        return f_neighbors

    # calculate path in the end
    def create_path(self):
        # now goal is start and start is goal
        s_node, g_node = self.goal, self.start
        path = []
        run = True
        while run:
            path.append(s_node)
            if s_node == g_node:
                return path
            else:
                s_node = self.cl_list[s_node].parrent


class Node(Grid):

    def __init__(self, f_value, g_value, node_heuristic, parrent=None):
        self.f_value = f_value
        self.g_value = g_value
        self.goal_node = None
        self.parrent = parrent
        self.heur = node_heuristic

    def __str__(self):
        return 'F: {0}, G: {1}, parrent {2}'.format(self.f_value, self.g_value, self.parrent)

    @staticmethod
    def heuristic(node, goal_node):
        # one step distance in four direction is 10, and in four diagonal is 14
        dist = 10
        dist_diag = 14
        dist_conts = dist_diag - dist
        dx = abs(node[0] - (goal_node[0]))  # 3
        dy = abs(node[1] - (goal_node[1]))  # 4
        return dist * max(dx, dy) + dist_conts * min(dx, dy)

    @staticmethod
    def g_value(node, parrent, parrent_g):
        parrent_x, parrent_y = parrent[0], parrent[1]
        node_x, node_y = node[0], node[1]
        if (node_x != parrent_x) and (node_y != parrent_y):
            return parrent_g + 14  # diagonal g = 14
        else:
            return parrent_g + 10  # strait g  = 10

    @classmethod
    def create_node(cls, node, node_g, parrent, parrent_g, goal_node):
        node_heuristic = Node.heuristic(node, goal_node)
        # TODO last heuristic value in object Node addet for debuging
        # TODO heuristic value shold be deleted
        return cls(node_heuristic + node_g, node_g, node_heuristic, parrent)


# activate and setup turtle library
turtle_pen = turtle.Turtle(visible=False)

turtle_pen.pen(fillcolor="black", pencolor="black", pensize=1)

turtle_screen = turtle.Screen()
turtle_screen.title('A* algoritam')
turtle_screen.setup(700, 700)
turtle_screen.tracer(False, delay=0)
# turtle.speed(4)
turtle_pen.hideturtle()

colors = {
    'white': '#ffffff',
    'black': '#000000',
    'orange': '#ff7200',
    'gray': '#515151',
    'darkred': '#af0909',
    'green': '#007b23',
    'yellow': '#fff000',
    'red': '#FF0000',
    'light_yellow': '#9dff70',
    'dark_yellow': '#fcb103'
}


def draw_box(dimension, color):
    turtle_pen.down()
    turtle_pen.begin_fill()
    turtle_pen.fillcolor(color)
    turtle_pen.setheading(270)
    turtle_pen.forward(dimension)
    turtle_pen.left(90)
    # 90 deg.
    turtle_pen.forward(dimension)
    turtle_pen.left(90)
    # 180 deg.
    turtle_pen.forward(dimension)
    turtle_pen.left(90)
    # 270 deg.
    turtle_pen.forward(dimension)
    turtle_pen.end_fill()
    turtle_pen.setheading(0)
    turtle_pen.up()
    turtle_pen.getscreen().update()


def draw_line(start, field_size, heading):
    turtle_pen.up()
    turtle_pen.setposition(start)
    turtle_pen.setheading(heading)
    turtle_pen.down()
    turtle_pen.forward(field_size)
    turtle_pen.up()
    turtle_pen.getscreen().update()
    # setposition for end - it is quicker

# draw grid of lines depending on grid size


def draw_lines(field_size, node_size):

    start_y = (field_size * node_size) / 2
    start_x = - (field_size * node_size) / 2
    for _ in range(field_size + 1):
        draw_line((start_x, start_y), field_size * node_size, 270)
        start_x += node_size
    start_y = (field_size * node_size) / 2
    start_x = - (field_size * node_size) / 2
    for _ in range(field_size + 1):
        draw_line((start_x, start_y), field_size * node_size, 0)
        start_y -= node_size


goal = (21, 42)
start = (21, 0)

wall = [(8.0, 8.0), (9.0, 8.0), (10.0, 8.0), (11.0, 8.0),
        (13.0, 8.0), (12.0, 8.0), (14.0, 8.0), (15.0, 8.0),
        (16.0, 8.0), (17.0, 8.0), (18.0, 8.0), (19.0, 8.0),
        (21.0, 8.0), (20.0, 8.0), (21.0, 9.0), (20.0, 9.0),
        (19.0, 9.0), (18.0, 9.0), (16.0, 9.0), (17.0, 9.0),
        (15.0, 9.0), (14.0, 9.0), (13.0, 9.0), (12.0, 9.0),
        (11.0, 9.0), (9.0, 9.0), (8.0, 9.0), (10.0, 9.0),
        (17.0, 12.0), (18.0, 12.0), (19.0, 12.0), (21.0, 12.0),
        (20.0, 12.0), (23.0, 12.0), (22.0, 12.0), (24.0, 12.0),
        (25.0, 12.0), (26.0, 12.0), (28.0, 12.0), (27.0, 12.0),
        (17.0, 13.0), (28.0, 11.0), (22.0, 16.0), (22.0, 17.0),
        (23.0, 17.0), (26.0, 17.0), (28.0, 17.0), (29.0, 17.0),
        (31.0, 17.0), (33.0, 17.0), (34.0, 17.0), (32.0, 17.0),
        (30.0, 17.0), (27.0, 17.0), (25.0, 17.0), (24.0, 17.0),
        (35.0, 17.0), (36.0, 17.0), (36.0, 16.0), (36.0, 15.0),
        (17.0, 22.0), (21.0, 22.0), (23.0, 22.0), (27.0, 22.0),
        (30.0, 22.0), (28.0, 22.0), (29.0, 22.0), (25.0, 22.0),
        (22.0, 22.0), (19.0, 22.0), (18.0, 22.0), (20.0, 22.0),
        (24.0, 22.0), (26.0, 22.0), (8.0, 28.0), (9.0, 28.0),
        (10.0, 28.0), (11.0, 28.0), (13.0, 28.0), (12.0, 28.0),
        (14.0, 28.0), (15.0, 28.0), (16.0, 28.0), (18.0, 28.0),
        (20.0, 28.0), (22.0, 28.0), (21.0, 28.0), (19.0, 28.0),
        (17.0, 28.0), (23.0, 33.0), (21.0, 33.0), (19.0, 33.0),
        (17.0, 33.0), (18.0, 33.0), (20.0, 33.0), (22.0, 33.0),
        (24.0, 33.0), (26.0, 33.0), (27.0, 33.0), (28.0, 33.0),
        (29.0, 33.0), (30.0, 33.0), (31.0, 33.0), (25.0, 33.0),
        (29.0, 27.0), (30.0, 27.0), (31.0, 27.0), (32.0, 26.0),
        (31.0, 26.0), (32.0, 27.0), (33.0, 26.0), (32.0, 25.0),
        (33.0, 25.0), (34.0, 26.0), (34.0, 25.0), (35.0, 25.0),
        (35.0, 26.0), (36.0, 24.0), (35.0, 24.0), (36.0, 25.0),
        (37.0, 23.0), (37.0, 24.0), (36.0, 23.0), (29.0, 37.0),
        (30.0, 37.0), (31.0, 37.0), (33.0, 37.0), (34.0, 37.0),
        (35.0, 37.0), (37.0, 37.0), (32.0, 37.0), (36.0, 37.0),
        (38.0, 37.0), (39.0, 37.0), (40.0, 37.0), (41.0, 37.0),
        (5.0, 39.0), (6.0, 39.0), (7.0, 38.0), (7.0, 39.0),
        (6.0, 38.0), (8.0, 38.0), (9.0, 38.0), (12.0, 38.0),
        (14.0, 38.0), (15.0, 38.0), (17.0, 38.0), (13.0, 38.0),
        (10.0, 38.0), (11.0, 38.0), (16.0, 38.0), (18.0, 38.0),
        (19.0, 38.0)]

node_size = 12
field_size = 50
# explanation of class
# Grid([number of nodes in x/y], [size of node *turtle draw], [start], [end])
terrain = Grid(field_size, node_size, start, goal)
terrain.wall = wall


turtle_pen.up()
draw_lines(field_size, node_size)


def nx(n): return (n - field_size / 2)


def ny(n): return (field_size / 2 -
                   n) if n < (field_size / 2) else - (n - field_size / 2)


# turtle_pen.setheading(0)
for i in range(field_size):
    for j in range(field_size):
        if (i, j) == start:
            turtle_pen.up()
            turtle_pen.setposition(nx(j) * node_size, ny(i) * node_size)
            draw_box(node_size, colors['green'])
        if (i, j) == goal:
            turtle_pen.up()
            turtle_pen.setposition(nx(j) * node_size, ny(i) * node_size)
            draw_box(node_size, colors['darkred'])
        if (i, j) in wall:
            # print('YES')
            turtle_pen.up()
            turtle_pen.setposition(nx(j) * node_size, ny(i) * node_size)
            # print((j - (field_size / 2)), (i + (field_size / 2)))
            # print(nx(j), ny(i))
            draw_box(node_size, colors['gray'])

terrain.op_list = {}
terrain.cl_list = {}
# start with start_node in terrain.op_list dictionary
H_start = Node.heuristic(terrain.start, terrain.goal)
terrain.op_list[terrain.start] = Node(H_start, 0, H_start, None)

current_node = terrain.goal

run = True
while run:

    # if open list empty there is no solution
    if not terrain.op_list:
        print(' No Solution! ')
        run = False
        break
    # find node with smallest F number
    curr_node_loc = min(terrain.op_list.items(), key=lambda x: x[1].f_value)[0]
    # if curent node location is goal node, problem solved
    if curr_node_loc == goal:
        print(' Path Found!')
        terrain.cl_list[curr_node_loc] = terrain.op_list.pop(curr_node_loc)
        if curr_node_loc != goal and curr_node_loc != start:
            turtle_pen.setposition(
                nx(curr_node_loc[1]) * node_size, ny(curr_node_loc[0]) * node_size)
            draw_box(node_size, colors['dark_yellow'])
        run = False
        break
    # current node goes from open to closed list
    terrain.cl_list[curr_node_loc] = terrain.op_list.pop(curr_node_loc)
    if curr_node_loc != goal and curr_node_loc != start:
        turtle_pen.setposition(
            nx(curr_node_loc[1]) * node_size, ny(curr_node_loc[0]) * node_size)
        draw_box(node_size, colors['dark_yellow'])

    for node in terrain.find_neighbors(curr_node_loc):
        # neighbors G value precalculated
        node_g_value = Node.g_value(
            node, curr_node_loc, terrain.cl_list[curr_node_loc].g_value)
        # if node is in CLOSED list
        if terrain.cl_list.get(node) != None:
            # AND if newly calculated G value is smaller then neighbors current value, update neighbor
            if node_g_value < terrain.cl_list[node].g_value:
                # update neighbor create_node(cls, node, node_g, parrent, parrent_g, goal_node)
                terrain.cl_list[node] = Node.create_node(
                    node, node_g_value, curr_node_loc, terrain.cl_list[curr_node_loc].g_value, goal)
        # if node is in OPEN list
        elif terrain.op_list.get(node) != None:
            # AND if newly calculated G value is smaller then neighbors current value, update neighbor
            if node_g_value < terrain.op_list[node].g_value:
                terrain.op_list[node] = Node.create_node(
                    node, node_g_value, curr_node_loc, terrain.cl_list[curr_node_loc].g_value, goal)
        else:
            terrain.op_list[node] = Node.create_node(
                node, node_g_value, curr_node_loc, terrain.cl_list[curr_node_loc].g_value, goal)
            # print(nx(node[0]), ny(node[1]))
            if node != goal:
                turtle_pen.setposition(
                    nx(node[1]) * node_size, ny(node[0]) * node_size)
                draw_box(node_size, colors['light_yellow'])

if terrain.op_list:
    x = terrain.create_path()
    print(x)
    for n in x:
        if n != goal and n != start:
            turtle_pen.setposition(nx(n[1]) * node_size, ny(n[0]) * node_size)
            draw_box(node_size, colors['red'])


turtle_screen.mainloop()
