#!/usr/bin/python3

import numpy as np
import math
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import rospy
from geometry_msgs.msg import Twist

class taara(object):
    def __init__(self, start, goal, wheel_rotation, clearance):
        self.start = start
        self.goal = goal
        self.xLength = 600
        self.yLength = 200
        self.wheel_rotation = wheel_rotation
        self.clearance = min(clearance, 25)
        self.radius = 10.0
        self.wheel_dist = 17.0
        self.wheel_rad = 1.9
        self.doori = {}
        self.path = {}
        self.c2c = {}
        self.c2g = {}
        self.hashMap = {}
        self.bachke = 15
        self.repeat = 100
    
    def boundary(self, current_x, current_y):
        node = (current_x >= (-50 + self.radius + self.clearance) and current_x <= (550 - self.radius - self.clearance) and current_y >= (-100 + self.radius + self.clearance) and current_y <= (100 - self.radius - self.clearance))
        return node

    def obstacle(self, row, col):
        total_clearance = self.clearance + self.radius
        diag_clearance = 1.4142 * total_clearance
        circle = 1
        rect1 = 1
        rect2 = 1

        circle = ((row - 350.0) * (row - 350.0) + (col - 10.0) * (col - 10.0)) - ((50 + total_clearance) * (50 + total_clearance))
        
        (x1, y1) = (100 - diag_clearance, -25 - diag_clearance)
        (x2, y2) = (115 + diag_clearance, -25 + diag_clearance)
        (x3, y3) = (115 + diag_clearance, 100)
        (x4, y4) = (100 - diag_clearance, 100)
        
        side1 = col-y1
        side2 = row-x2
        side3 = col-y3
        side4 = row-x4
        if(side1 >= 0 and side2 <= 0 and side3 <= 0 and side4 >= 0):
            rect1 = 0

        (x1, y1) = (200 + diag_clearance, -100)
        (x2, y2) = (215 + diag_clearance, -100)
        (x3, y3) = (215 - diag_clearance, 25 + diag_clearance)
        (x4, y4) = (200 - diag_clearance, 25 - diag_clearance)
        side1 = col-y1
        side2 = row-x2
        side3 = col-y3
        side4 = row-x4
        if(side1 >= 0 and side2 <= 0 and side3 <= 0 and side4 >= 0):
            rect2 = 0
        
        if(circle <= 0 or rect1 == 0 or rect2==0):
            return True
        return False
    
    def animate(self, gelelo, parat):
        x_start = []
        y_start = []
        x_end = []
        y_end = []
        explored_x_start = []
        explored_y_start = []
        explored_x_end = []
        explored_y_end = []
        fig, ax = plt.subplots()
        plt.xlabel("x-coordinate(in m)")
        plt.ylabel("y-coordinate(in m)")
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(-50/100, 550/100)
        plt.ylim(-100/100, 100/100)
        counter = 0

        x_obstacle = []
        y_obstacle = []
        size = []

        for i in range(-50,550):
            for j in range(-100,100):
                if(self.obstacle(i, j)):
                    x_obstacle.append(i / 100.0)
                    y_obstacle.append(j / 100.0)     
                    size.append(15)      
        x_obstacle = np.array(x_obstacle)
        y_obstacle = np.array(y_obstacle)
        plt.scatter(x_obstacle, y_obstacle, color='r', s=size)

        for i in range(1, len(gelelo)):
            parentNode = self.path[gelelo[i]][0]
            explored_x_start.append(parentNode[0] / 100.0)
            explored_y_start.append(parentNode[1] / 100.0)
            explored_x_end.append((gelelo[i][0] - parentNode[0]) / 100.0)
            explored_y_end.append((gelelo[i][1] - parentNode[1]) / 100.0)    
            
            if(counter % 2010 == 0):
                plt.quiver(np.array((explored_x_start)), np.array((explored_y_start)), np.array((explored_x_end)), np.array((explored_y_end)), units = 'xy', scale = 1, color = 'g')
            counter = counter + 1

        if(len(parat) > 0):
            for i in range(1, len(parat)):
                x_start.append(parat[i-1][0] / 100.0)
                y_start.append(parat[i-1][1] / 100.0)
                x_end.append((parat[i][0] - parat[i-1][0]) / 100.0)
                y_end.append((parat[i][1] - parat[i-1][1]) / 100.0)  
                if(counter % 6 == 0):
                    plt.quiver(np.array((x_start)), np.array((y_start)), np.array((x_end)), np.array((y_end)), units = 'xy', scale = 0.6, color = 'b')
                counter = counter + 1

        plt.legend()
        plt.show()
    
    def gazebo_velocity_update(self, currentNode, leftRPM, rightRPM):
        left_wheel_vel = leftRPM * 2 * np.pi / 60.0
        right_wheel_vel = rightRPM * 2 * np.pi / 60.0
        dx = 0
        dy = 0
        dtheta = 0
        cost = 0
        flag = True
        x = currentNode[0]
        y = currentNode[1]
        theta = currentNode[2]
        
        w = (self.wheel_rad / self.wheel_dist) * (right_wheel_vel - left_wheel_vel)
        
        for i in range(0, self.repeat):
            dvx = self.wheel_rad * 0.5 * (left_wheel_vel + right_wheel_vel) * math.cos(theta)
            dvy = self.wheel_rad * 0.5 * (left_wheel_vel + right_wheel_vel) * math.sin(theta)
            dx = dvx / self.repeat
            dy = dvy / self.repeat
            dtheta = w / self.repeat
            x = x + dx
            y = y + dy
            theta = theta + dtheta
            cost = cost + np.sqrt(dx ** 2 + dy ** 2)
            
            if(self.boundary(x, y) == False or self.obstacle(x, y)):
                flag = False
                break
                
        if(flag != False and self.hashMap.get(int(int(x * 100) + int(y * 10))) != None):
            flag = False
            
        return (x, y, theta, cost, dvx, dvy, w, flag)

    def action_feasibility_check(self, currentNode, leftRPM, rightRPM):
        (new_x, new_y, new_theta, cost, dvx, dvy, dw, flag) = self.gazebo_velocity_update(currentNode, leftRPM, rightRPM)
        self.hashMap[int(int(new_x * 100) + int(new_y * 10))] = 1
        if(flag == True and self.boundary(new_x, new_y) and self.obstacle(new_x, new_y) == False):
            return (True, new_x, new_y, new_theta, dvx, dvy, dw, cost)
        return (False, new_x, new_y, new_theta, dvx, dvy, dw, cost)

    def update_action_variables(self, currentNode, w, new_x, new_y, new_theta, action, cost):
        new_c2c = self.c2c[currentNode] + w
        new_c2g = self.heuristic(new_x, new_y)
        newDistance = new_c2c + new_c2g + cost

        if(self.doori.get((new_x, new_y, new_theta)) == None):
            self.doori[(new_x, new_y, new_theta)] = float('inf')  

        if(self.doori[(new_x, new_y, new_theta)] > newDistance):
            self.doori[(new_x, new_y, new_theta)] = newDistance
            self.c2c[(new_x, new_y, new_theta)] = new_c2c
            self.c2g[(new_x, new_y, new_theta)] = new_c2g
            self.path[(new_x, new_y, new_theta)] = (currentNode, action)
            return True
        return False
        
    def heuristic(self, current_x, current_y, w = 3.0):
        return w * (np.sqrt(((self.goal[0] - current_x) ** 2) + ((self.goal[1] - current_y) ** 2)))
    
    # a-star algo
    def aStar(self):
        # mark source node and create a queue
        gelelo = []
        queue = []
        self.c2c[self.start] = 0
        self.c2g[self.start] = self.heuristic(self.start[0], self.start[1])
        self.doori[self.start] = self.c2c[self.start] + self.c2g[self.start]
        heappush(queue, (self.doori[self.start], self.c2c[self.start], self.start))
        backtrackNode = None
        flag = 0
        steps = 0
        
        # run A-star
        while(len(queue) > 0):
            
            _, _, currentNode = heappop(queue) 
            self.hashMap[int(int(currentNode[0] * 100) + int(currentNode[1] * 10))] = 1
            gelelo.append(currentNode)
            steps = steps + 1
            
            if(np.square(np.abs(currentNode[0] - self.goal[0])) + np.square(np.abs(currentNode[1] - self.goal[1])) < self.bachke):
                backtrackNode = currentNode
                flag = 1
                break
               
            if(steps > 1000000):
                break

            (action1, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, 0, self.wheel_rotation[0])
            if(action1):
                heap_upd = self.update_action_variables(currentNode, self.wheel_rotation[0], new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action2, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[0], 0)
            if(action2):
                heap_upd = self.update_action_variables(currentNode, self.wheel_rotation[0], new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action3, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[0], self.wheel_rotation[0])
            if(action3):
                heap_upd = self.update_action_variables(currentNode, (self.wheel_rotation[0] * 1.4142), new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action4, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, 0, self.wheel_rotation[1])      
            if(action4):
                heap_upd = self.update_action_variables(currentNode, self.wheel_rotation[1], new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action5, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[1], 0)
            if(action5):
                heap_upd = self.update_action_variables(currentNode, self.wheel_rotation[1], new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action6, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[1], self.wheel_rotation[1])
            if(action6):
                heap_upd = self.update_action_variables(currentNode, (self.wheel_rotation[1] * 1.4142), new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action7, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[0], self.wheel_rotation[1])
            if(action7):
                heap_upd = self.update_action_variables(currentNode, max((self.wheel_rotation[0] * 1.4142), (self.wheel_rotation[1] * 1.4142)), new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

            (action8, new_x, new_y, new_theta, dvx, dvy, dw, cost) = self.action_feasibility_check(currentNode, self.wheel_rotation[1], self.wheel_rotation[0])
            if(action8):
                heap_upd = self.update_action_variables(currentNode, max((self.wheel_rotation[0] * 1.4142), (self.wheel_rotation[1] * 1.4142)), new_x, new_y, new_theta, (dvx, dvy, dw), cost)
                if(heap_upd):
                    heappush(queue, (self.doori[(new_x, new_y, new_theta)], self.c2c[(new_x, new_y, new_theta)], (new_x, new_y, new_theta)))

        if(flag == 0):
            return (gelelo, [], float('inf'))
        
        parat = []
        actions = []
        node = backtrackNode
        action = None
        while(node != self.start):
            parat.append(node)
            if(action != None):
                actions.append(action)
            node, action = self.path[node]
        parat.append(self.start)
        actions.append(action)
        parat = list(reversed(parat))  
        actions = list(reversed(actions)) 
        return (gelelo, parat, actions, self.doori[backtrackNode])

def update_gazebo_pose(pub_vel, dvx, dvy, dw):
    r = rospy.Rate(100)
    vel_value = Twist()
    velocity = np.sqrt(dvx * dvx + dvy * dvy) / 100.0
    endTime = rospy.Time.now() + rospy.Duration(1)
    while rospy.Time.now() < endTime:
        vel_value.linear.x = velocity
        vel_value.angular.z = dw
        pub_vel.publish(vel_value)
        r.sleep()

rospy.init_node('astar_turtlebot3')
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.sleep(1)

while(1): 

    x_start = 0.0
    y_start = 0.0
    startOrientation = 0.0
    X_goal = 3.5
    Y_goal = 0.8
    left_RPM = 100.0
    right_RPM = 50.0
    clearance = 0.05

    # print("Please enter the same starting coordinates as that of argument passed\n")
    # x_start = float(input("Enter the x-coordinate for start node(in m) : ")) # sample - 0
    # y_start = float(input("Enter the y-coordinate for start node(in m) : ")) # sample - 0
    # startOrientation = float(input("Enter the orientation for start node : ")) # sample - 0
    # X_goal = float(input("Enter the x-coordinate for goal node(in m) : ")) # sample - 3.5
    # Y_goal = float(input("Enter the y-coordinate for goal node(in m) : ")) # sample - 0.8
    # left_RPM = float(input("Enter the side1 value of RPM : ")) # sample - 100
    # right_RPM = float(input("Enter the side2 value of RPM : ")) # sample - 50
    # clearance = float(input("Enter the clearance of the rigid robot(in m) : ")) # sample - 0.05

    # take start and goal node as input
    start = (x_start * 100.0, y_start * 100.0, startOrientation)
    goal = (X_goal * 100.0, Y_goal * 100.0)
    wheel_rotation = (left_RPM, right_RPM)
    astar = taara(start, goal, wheel_rotation, clearance * 50)

    if(astar.boundary(start[0], start[1])):
        if(astar.boundary(goal[0], goal[1])):
            if(astar.obstacle(start[0],start[1]) == False):
                if(astar.obstacle(goal[0], goal[1]) == False):
                    print("\nBeginning to plan using astar, might take a while, please wait!\n")
                    states = astar.aStar()
                    explored_states = states[0]
                    backtrack_states = states[1]
                    actions = states[2]

                    if(len(backtrack_states) == 0):
                        print("No optimal path.")
                        break
                    else:
                        print("Optimal path found\n")

                        for i in range(0, len(actions)):
                            dvx, dvy, dw = actions[i]
                            update_gazebo_pose(pub_vel, dvx, dvy, dw) 
                        
                        # stop once goal is reached
                        update_gazebo_pose(pub_vel, 0,0,0)

                        print("Creating 2D Map, please wait for sometime\n")
                        astar.animate(explored_states,  backtrack_states)
                        print("Finished Creating 2D Map\n")
                        break
                else:
                    print("goal node in obstacle space")
            else:
                print("start node in obstacle space")
        else:
            print("goal node outside map")
    else:
        print("start node outside map")