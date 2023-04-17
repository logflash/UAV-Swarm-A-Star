# IAN HENRIQUES - 2019-2020

from math import *
from pygame import *
from random import *
from heapq import *
init();

global fps, scale, reach, angles, radius, visible;
fps = 10; # refresh rate of the simulation
sc = 10; # scale of screen size (in pixels) to the actual coordinate system
reach = 100; # how far the drone can see
sx = 500; # screen size (X)
sy = 500; # screen size (Y)
randlock = 60; # to prevent the motion from being too unpredictable
angles = 24; # the number of vision angles for the drone
visible = True;

'''

Energy efficiency calculations

Assumptions made:
* acceleration is the overall acceleration, not the thrust
* the rotors never shut off to coast to a stop using drag
* quadrotor drone only
* momentum theory holds for these drones

'''

D = 0.254 # diameter of rotor blades
radius = D; # the radius of the drone is approx diameter of one rotor
dens = 1.225 # air density (approximate)
m = 2.57 # mass of drone (kg)

def power(v,a): # power of the aircraft at certain flight conditions
    F_d = dens/2 * (1.49 * 0.0599 + 2.2 * 0.0135 + 1*0.0037) * v**2
    T = hypot(F_d + m*a, 9.81*m)
    alpha = atan2(F_d + m*a, 9.81*m)
    C = (2 * T / (4 * pi * D**2 * dens))**2
    sa = sin(alpha)
    f = lambda vi: vi**4 - 2 * vi**3 * v * sa + vi**2 * v**2 - C
    Df = lambda vi: 4 * vi**3 - 6 * vi**2 * v * sa + 2 * vi * v**2
    vi = 140
    while True:
        fvi, Dfvi = f(vi), Df(vi)
        if abs(fvi) < 1e-10:
            if vi > 0: break
            else: vi = abs(vi)
        vi = vi - fvi/Dfvi
    return T*(vi+v*sa)

def epd(v,a): # energy per distance
    P = power(v,a)
    if v > 0: return P/v
    return int(1e15) # a big number

def epf(v,a): # energy per frame
    P = power(v,a)
    return P/fps

def epf2(vx,vy,ax,ay): # energy per frame with rectangular coords
    v = hypot(vx,vy);
    a = hypot(ax,ay);
    return epf(v,a);

'''

Entity class to model any object on the screen

Assumptions made:
* Randomly switches directions (if it's an obstacle) at a fixed rate
* Moves according to the laws of kinematics

'''

class entity:
    def __init__(self, screen, x, y, r):
        self.x = x; self.y = y; self.r = r; self.screen = screen;
        self.startx = x; self.starty = y;
        self.vx = 0; self.vy = 0; self.ax = 0; self.ay = 0;
        self.color = [255,100,100]; self.randtick = randlock+1;
        self.crashed = False; self.success = False;
    def plot(self):
        if (not visible): return;
        draw.circle(self.screen, self.color, (int(self.x*sc), int(self.y*sc)), int(self.r*sc), 0);
    def randspeed(self):
        if self.randtick > randlock:
            v = random()*5;
            t = random()*2*pi;
            self.vx = v*cos(t);
            self.vy = v*sin(t);
            self.randtick = 0;
        else:
            self.randtick += 1;
    def move(self):
        self.vx += (self.ax) * (1/fps);
        self.vy += (self.ay) * (1/fps);
        self.ax = 0; self.ay = 0;
        v = hypot(self.vx, self.vy);
        if (v > 20):
            self.vx = self.vx * 20 / v;
            self.vy = self.vy * 20 / v;
        if (not self.crashed) and (not self.success):
            self.x += self.vx * (1/fps);
            self.y += self.vy * (1/fps);
            if (self.x < 0): self.crashed = True;
            elif (self.y < 0): self.crashed = True;
            elif (self.x*sc > sx): self.crashed = True;
            elif (self.y*sc > sy): self.crashed = True;
            if (self.crashed): self.color = [255,0,0];
            if (isinstance(self, player)):
                self.goalcheck();
                self.used += epf2(self.vx, self.vy, self.ax, self.ay);
        if (self.crashed) and (not isinstance(self, player)):
            if (self.x < 0): self.x = sx/sc - self.r;
            if (self.y < 0): self.y = sy/sc - self.r;
            if (self.x*sc > sx): self.x = self.r;
            if (self.y*sc > sy): self.y = self.r;
            self.crashed = False; self.color = [255,100,100];

'''

Player class to simulate a drone

Assumptions made:
* Fixed collections per frame (due to LiDAR)
* Fixed max speed

'''

class player(entity):
    def __init__(self, screen, x, y, r, gx, gy, vis=False):
        entity.__init__(self, screen, x, y, r);
        self.gx = gx;
        self.gy = gy;
        self.inputs = angles;
        self.reset();
        self.color = [255,255,0];
        self.used = 0;
        self.vis = vis;
        # stores the inputs the algorithm decides
        self.up = False;
        self.down = False;
        self.left = False;
        self.right = False;
    def reset(self):
        self.d = [1.0 for i in range(self.inputs)]; # resets input function to full extension of rays
    def plot(self):
        if (not visible):
            self.reset();
            return;
        draw.circle(self.screen, [self.color[0]/3, self.color[1]/3, self.color[2]/3], (int(self.gx*sc), int(self.gy*sc)), int(1.5*sc), 0);
        # have to plot all vision lines
        if ((not self.success) and (not self.crashed)):
            if (self.vis):
                for i in range(self.inputs):
                    shade = int(256-256*self.d[i]);
                    if (shade):
                        draw.line(self.screen, [shade,shade,shade], (int(self.x*sc), int(self.y*sc)), (int(self.x*sc + sc*(reach*self.d[i])*cos(i*pi/self.inputs*2)), int(self.y*sc - sc*(reach*self.d[i])*sin(i*pi/self.inputs*2))), 3);
            entity.plot(self);
            if ((self.vx != 0) or (self.vy != 0)):
                draw.line(self.screen, self.color, (int(self.x*sc), int(self.y*sc)), (int(self.x*sc + self.vx*sc), int(self.y*sc + self.vy*sc)), 2);
        else:
            entity.plot(self);
        self.reset();
    def update(self, inp, dist, obst=True):
        if (obst): # blocking object is an obstacle
            self.d[int(inp)] = min(self.d[int(inp)], dist/reach);
        else: # blocking object is a wall
            self.dwall[int(inp)] = min(self.dwall[int(inp)], dist/reach);
    def detect(self, other):
        dx = other.x - self.x; # distance x to get to object
        dy = self.y - other.y; # distance y to get to object
        l = hypot(dx, dy);
        if (l <= self.r + other.r):
            self.color = [255,0,0]; self.crashed = True; self.ax = 0; self.ay = 0;
        t = 0;
        if (dx != 0): t = atan(dy/dx);
        elif (dy > 0): t = pi/2;
        elif (dy < 0): t = -pi/2;
        if (dx < 0): t += pi;
        if (t < 0): t += 2*pi;
        if (other.r > l): return;
        dt = asin(other.r/l);
        bmin = (t-dt) // (pi/self.inputs*2); # lowest bin that an obstacle would fall into
        bmax = (t+dt) // (pi/self.inputs*2); # highest bin that an obstacle would fall into
        if (bmax > bmin):
            for b in range(int(bmin+1), int(bmax+1)):
                alpha = t - b*(pi/self.inputs*2);
                if ((other.r)**2 - (l*sin(alpha))**2 > 0):
                    self.update(b%self.inputs, (l*cos(alpha)-sqrt((other.r)**2-(l*sin(alpha))**2)));
    def control(self):
        # take in inputs, the algorithms that inherit will ultimately decide this
        self.up = key.get_pressed()[K_UP];
        self.down = key.get_pressed()[K_DOWN];
        self.left = key.get_pressed()[K_LEFT];
        self.right = key.get_pressed()[K_RIGHT];
    def act(self):
        ''' USELESS CODE SNIPPET
        if (not(self.up or self.down or self.left or self.right or self.vx != 0 or self.vy != 0)):
            self.right = True;
        '''
        # based on inputs, decide accelerations
        if (self.up):    self.ay = -5;
        if (self.down):  self.ay = 5;
        if (self.left):  self.ax = -5;
        if (self.right): self.ax = 5;
        if (hypot(self.ax, self.ay) > 5):
            self.ax *= 0.7; self.ay *= 0.7;
    def goalcheck(self):
        # check if it has reached within radius of the goal
        dx = self.gx - self.x;
        dy = self.y - self.gy;
        l = hypot(dx, dy);
        if (l <= 1.5):
            self.success = True; self.color = [0,255,0];

'''

Design of A* algorithm

Assumptions made:
* The drone will move away from the closest obstacles
* The heuristic cost is just the Euclidean distance to the goal
* Since the map is constantly changing, small fixed-step iterations of A* will be done

Options for acceleration are organized as follows:
2 3 4
5 1 6
7 8 9

'''

penalties = (lambda x: 0, lambda x: 25 - x/2, lambda x: 25 / x);
pfuncs = ("p = 0", "p = 25 - x/2", "p = 25 / x");

options = ((0,0,0,0), (0,0,0,0),
           (1,0,1,0), (1,0,0,0), (1,0,0,1),
           (0,0,1,0),            (0,0,0,1),
           (0,1,1,0), (0,1,0,0), (0,1,0,1));

class state:
    def __init__(self, seq, seq_len, x, y, vx, vy, g, f):
        self.seq = seq; self.seq_len = seq_len;
        self.x = x; self.y = y;
        self.vx = vx; self.vy = vy;
        self.g = g; self.f = f;
    def __lt__(self, other):
        return self.f < other.f;  

class playerAStar(player):
    def __init__(self, screen, x, y, r, gx, gy, pathlock, pfunc, vis=False):
        player.__init__(self, screen, x, y, r, gx, gy, vis);
        self.pathlock = pathlock;
        self.p = penalties[pfunc];
        self.chosen_seq = 0;
        self.d_rect = [0 for i in range(2*angles)];
    def sense(self):
        for i in range(angles):
            alpha = 2*pi*i/self.inputs;
            self.d_rect[2*i] = reach * self.d[i] * cos(alpha) + self.x;
            self.d_rect[2*i+1] = - reach * self.d[i] * sin(alpha) + self.y;
    def parse(self, option, stateIn):
        # Unpack the current state
        seq = stateIn.seq;
        seq_len = stateIn.seq_len;
        x_curr = stateIn.x;
        y_curr = stateIn.y;
        vx_curr = stateIn.vx;
        vy_curr = stateIn.vy;
        g_curr = stateIn.g;
        # Make a decision and apply kinematics
        ax_pred = 0; ay_pred = 0;
        if (option == 2): ax_pred, ay_pred = -3.5, -3.5;
        if (option == 3): ax_pred, ay_pred = 0,    -5;
        if (option == 4): ax_pred, ay_pred = 3.5,  -3.5;
        if (option == 5): ax_pred, ay_pred = -5,   0;
        if (option == 6): ax_pred, ay_pred = 5,    0;
        if (option == 7): ax_pred, ay_pred = -3.5, 3.5;
        if (option == 8): ax_pred, ay_pred = 0,    5;
        if (option == 9): ax_pred, ay_pred = 3.5,  3.5;
        vx_pred = vx_curr + ax_pred / fps; vy_pred = vy_curr + ay_pred / fps;
        x_pred = x_curr + vx_pred / fps; y_pred = y_curr + vy_pred / fps;
        # G cost - cost to reach the next spot
        g_pred = g_curr + epf2(vx_pred, vy_pred, 0, 0);
        # Check if it's too close to where the obstacles currently are
        penalty = 0;
        for i in range(angles):
            close = hypot(x_pred - self.d_rect[2*i], y_pred - self.d_rect[2*i+1]);
            if (close < 5):
                penalty += self.p(close);
        # Check if it would hit the wall
        if (x_pred < 0): return None;
        elif (y_pred < 0): return None;
        elif (x_pred*sc > sx): return None;
        elif (y_pred*sc > sy): return None;
        # H cost - cost to reach the goal from the next spot
        h_pred = hypot(self.gx - x_pred, self.gy - y_pred);
        # F cost - sum of all costs
        f_pred = g_pred + h_pred + penalty;
        # Return the new system state
        return state(option * 10**seq_len + seq, seq_len+1, x_pred, y_pred, vx_pred, vy_pred, g_pred, f_pred);
    def aStar(self):
        # Keep a priority queue of states
        pq = [state(0, 0, self.x, self.y, self.vx, self.vy, 0, 0)];
        while True:
            # Remove and store the lowest cost state
            if (len(pq)): curr_state = heappop(pq);
            else: break;
            if (visible): draw.circle(self.screen, [255,0,0], (int(curr_state.x*sc), int(curr_state.y*sc)), 1, 0);
            if (curr_state.seq_len >= self.pathlock):
                self.chosen_seq = curr_state.seq;
                return;
            for i in range(1,10):
                next_state = self.parse(i, curr_state);
                if (next_state): heappush(pq, next_state);
            if (visible): display.update();
    def control(self):
        safe = not(any([i-1 for i in self.d]));
        if (self.chosen_seq == 0):
            self.sense();
            # Sense and destroy orbits
            goal = hypot(self.x - self.gx, self.y - self.gy);
            v = hypot(self.vx, self.vy);
            vnew = v - 5 / fps;
            ca = 1;
            if (goal and v):
                ca = abs((self.x - self.gx) * self.vx + (self.y - self.gy) * self.vy) / goal / v;
            if (ca < 0.2 and safe):
                self.vx *= (vnew / v);
                self.vy *= (vnew / v);
                if (vnew < 2.5 / fps):
                    self.vx = 0;
                    self.vy = 0;
                return;
            # Run A* if all is well
            self.aStar();
        option = self.chosen_seq % 10;
        self.chosen_seq = int((self.chosen_seq - option)/10);
        self.up = options[option][0];
        self.down = options[option][1];
        self.left = options[option][2];
        self.right = options[option][3];
        self.act();

'''

Main

Assumptions made:
* 100 m reach
* 5 m/s/s max acceleration
* Acceleration is in x and y directions for easy control

Modes:
0 - no obstacles
1 - 1 stat
2 - 2 birds 1 building
3 - 4 birds
4 - 8 birds
5 - 12 birds
6 - 16 birds
7 - 5 copies

'''
        
def main(g, m, p, n, s):
    
    global angles;
    angles = n;

    if (m == 7): return main_swarm(g, m, p, s);
    
    screen = display.set_mode([sx,sy]);
    seed(g);
    startx = randint(1,int(sx/sc-1));
    starty = randint(1,int(sy/sc-1));
    gx = randint(1,int(sx/sc-1));
    gy = randint(1,int(sy/sc-1));
    halfx = sx/sc/2;
    halfy = sy/sc/2;
    
    x = playerAStar(screen, startx, starty, radius, gx, gy, s, p, True);
    obstacle_list = [];
    if (m == 2):
        obstacle_list.append(entity(screen, halfx, halfy, 0.3)); # bird
        obstacle_list.append(entity(screen, halfx, halfy, 0.5)); # large drone
    if (m == 1 or m == 2):
        obstacle_list.append(entity(screen, halfx, halfy, 3)); # stationary obstacle
    if (m == 3):
        for i in range(4):
            obstacle_list.append(entity(screen, halfx, halfy, 0.3)); # bird
    if (m == 4):
        for i in range(8):
            obstacle_list.append(entity(screen, halfx, halfy, 0.3)); # bird
    if (m == 5):
        for i in range(12):
            obstacle_list.append(entity(screen, halfx, halfy, 0.3)); # bird
    if (m == 6):
        for i in range(16):
            obstacle_list.append(entity(screen, halfx, halfy, 0.3)); # bird

    frame = 0;
    if (not visible): quit();
    
    while True:
        if (visible):
            for e in event.get():
                if e.type == QUIT:
                    quit(); return;
                if e.type == KEYDOWN:
                    if e.key == K_ESCAPE:
                        quit(); return;
        if (x.crashed):
            if (not hypot(gx-startx, gy-starty)): return "err";
            quit(); return -hypot(x.gx-x.x, x.gy-x.y)/hypot(gx-startx, gy-starty); # ratio of final to init distance
        if (x.success):
            if (not x.used): return "err";
            quit(); return 21.5554*hypot(gx-startx, gy-starty)/x.used; # ratio of expected to observed
        if (frame > 300):
            return 0; # took too long but didn't crash
        for o in obstacle_list:
            x.detect(o);
        x.control();
        if (visible): screen.fill([0,0,0]);
        x.move();
        x.plot();
        for o in obstacle_list:
            if (o.r < 2): o.randspeed();
            o.move();
            o.plot();
        if (visible): display.update();
        time.Clock().tick(fps);
        frame += 1;

def main_swarm(g, m, p, s):    

    screen = display.set_mode([sx,sy]);
    seed(g);

    startx1 = randint(1,int(sx/sc-1));
    starty1 = randint(1,int(sy/sc-1));
    gx1 = randint(1,int(sx/sc-1));
    gy1 = randint(1,int(sy/sc-1));
    
    startx2 = randint(1,int(sx/sc-1));
    starty2 = randint(1,int(sy/sc-1));
    gx2 = randint(1,int(sx/sc-1));
    gy2 = randint(1,int(sy/sc-1));
    
    startx3 = randint(1,int(sx/sc-1));
    starty3 = randint(1,int(sy/sc-1));
    gx3 = randint(1,int(sx/sc-1));
    gy3 = randint(1,int(sy/sc-1));

    startx4 = randint(1,int(sx/sc-1));
    starty4 = randint(1,int(sy/sc-1));
    gx4 = randint(1,int(sx/sc-1));
    gy4 = randint(1,int(sy/sc-1));

    startx5 = randint(1,int(sx/sc-1));
    starty5 = randint(1,int(sy/sc-1));
    gx5 = randint(1,int(sx/sc-1));
    gy5 = randint(1,int(sy/sc-1));

    startx6 = randint(1,int(sx/sc-1));
    starty6 = randint(1,int(sy/sc-1));
    gx6 = randint(1,int(sx/sc-1));
    gy6 = randint(1,int(sy/sc-1));
    
    drone_list = [playerAStar(screen, startx1, starty1, radius, gx1, gy1, s, p, False),
                  playerAStar(screen, startx2, starty2, radius, gx2, gy2, s, p, False),
                  playerAStar(screen, startx3, starty3, radius, gx3, gy3, s, p, False),
                  playerAStar(screen, startx4, starty4, radius, gx4, gy4, s, p, False),
                  playerAStar(screen, startx5, starty5, radius, gx5, gy5, s, p, False),
                  playerAStar(screen, startx6, starty6, radius, gx6, gy6, s, p, False)];

    drone_list[0].color = [0,0,255];
    drone_list[1].color = [0,255,255];
    drone_list[2].color = [255,0,255];
    drone_list[3].color = [255,255,0];
    drone_list[4].color = [255,255,255];
    drone_list[5].color = [255,174,0];

    if (not visible): quit();
    
    frame = 0;
    results = [0,0,0,0,0,0];
    
    while True:
        if (visible):
            for e in event.get():
                if e.type == QUIT:
                    quit(); return;
                if e.type == KEYDOWN:
                    if e.key == K_ESCAPE:
                        quit(); return;

        for i in range(0, len(drone_list)):
            for j in range(i+1, len(drone_list)):
                if (not (drone_list[i].crashed or drone_list[j].crashed or drone_list[i].success or drone_list[j].success)):
                    drone_list[i].detect(drone_list[j]);
                    drone_list[j].detect(drone_list[i]);
        
        for i in range(len(drone_list)):
            if (drone_list[i].crashed):
                if (not hypot(drone_list[i].gx-drone_list[i].startx, drone_list[i].gy-drone_list[i].starty)): return "err";
                results[i] = - hypot(drone_list[i].gx-drone_list[i].x, drone_list[i].gy-drone_list[i].y) / hypot(drone_list[i].gx-drone_list[i].startx, drone_list[i].gy-drone_list[i].starty);
            if (drone_list[i].success):
                if (not drone_list[i].used): return "err";
                results[i] = 21.5554*hypot(drone_list[i].gx-drone_list[i].startx, drone_list[i].gy-drone_list[i].starty) / drone_list[i].used;
            if (not (drone_list[i].crashed or drone_list[i].success)):
                drone_list[i].control();
                drone_list[i].move();
                if (frame > 300): results[i] = 0;

        if (visible): screen.fill([0,0,0]);
        
        for i in range(len(drone_list)):
            drone_list[i].plot();

        if (visible): display.update();

        if (all(results)): break;
        time.Clock().tick(fps);
        frame += 1;
    
    quit(); return results;

# Auxiliary function for interpreting the lists returned by main_swarm
def avg():
    while True:
        i = input();
        if (i):
            i = eval(i);
            print(sum(i)/len(i));

def RUN():
    for step in [1,3,5]:
        print("********\nSTEP SIZE = " + str(step));
        for num_inp in [12, 24, 36]:
            print("&&&&&&&&&\nInputs = " + str(num_inp));
            for pen in [0, 1, 2]: # Checks all 3 inputs
                if (num_inp == 24 and pen == 0): continue;
                if (num_inp == 36 and pen == 0): continue;
                print("^^^^^^^^\nPenalty Function: " + pfuncs[pen]);
                for mode in range(8):
                    print("%%%%%%%%\nmode = " + str(mode));
                    for gen in range(15):
                        print(main(gen, mode, pen, num_inp, step));
