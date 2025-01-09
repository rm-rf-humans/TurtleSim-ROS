#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class AdvancedTurtleBouncer:
    def __init__(self):
        rospy.init_node('advanced_turtle_bouncer', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        self.current_pose = None
        self.base_speed = 2.0  
        
        self.min_x, self.max_x = 0.5, 11.5
        self.min_y, self.max_y = 0.5, 11.5
        
        self.bounce_margin = 0.6 
        
        self.bounce_angle = random.uniform(0, 2 * math.pi)
        
        self.rate = rospy.Rate(10)  

        self.positions = []

        self.positions_lock = threading.Lock()

        self.fig, self.ax = plt.subplots()
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=100)
    
    def pose_callback(self, pose):
        self.current_pose = pose
        with self.positions_lock:
            self.positions.append((pose.x, pose.y))
    
    def adjust_bounce_angle(self):
        if self.current_pose.x <= (self.min_x + self.bounce_margin):
            print("Bouncing off LEFT wall!")
            self.bounce_angle = random.uniform(-math.pi/4, math.pi/4)
        
        elif self.current_pose.x >= (self.max_x - self.bounce_margin):
            print("Bouncing off RIGHT wall!")
            self.bounce_angle = random.uniform(math.pi * 3/4, math.pi * 5/4)
        
        if self.current_pose.y <= (self.min_y + self.bounce_margin):
            print("Bouncing off BOTTOM wall!")
            self.bounce_angle = random.uniform(math.pi/4, math.pi * 3/4)
        
        elif self.current_pose.y >= (self.max_y - self.bounce_margin):
            print("Bouncing off TOP wall!")
            self.bounce_angle = random.uniform(math.pi * 5/4, math.pi * 7/4)
    
    def move(self):
        while self.current_pose is None and not rospy.is_shutdown():
            self.rate.sleep()
        
        vel_msg = Twist()
        
        while not rospy.is_shutdown():
            self.adjust_bounce_angle()
            
            vel_msg.linear.x = self.base_speed * math.cos(self.bounce_angle)
            vel_msg.linear.y = self.base_speed * math.sin(self.bounce_angle)
            
            self.velocity_publisher.publish(vel_msg)
            
            self.rate.sleep()
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)

    def update_plot(self, frame):
        self.ax.clear()
        with self.positions_lock:
            x_vals = [pos[0] for pos in self.positions]
            y_vals = [pos[1] for pos in self.positions]
        min_length = min(len(x_vals), len(y_vals))
        x_vals = x_vals[:min_length]
        y_vals = y_vals[:min_length]
        self.ax.plot(x_vals, y_vals, 'b-')
        self.ax.set_xlim(self.min_x - 1, self.max_x + 1)
        self.ax.set_ylim(self.min_y - 1, self.max_y + 1)
        self.ax.set_title('Turtle Trajectory')
    
    def start_plot(self):
        plot_thread = threading.Thread(target=plt.show)
        plot_thread.start()
    
    def run(self):
        self.start_plot()
        try:
            self.move()
        except rospy.ROSInterruptException:
            pass

def main():
    bouncer = AdvancedTurtleBouncer()
    bouncer.run()

if __name__ == '__main__':
    main()
