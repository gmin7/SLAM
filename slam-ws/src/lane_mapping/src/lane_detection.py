#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from lane_pipeline import get_lane_points


# Define the class that will store global state of the robot
class Nav_State:
    def __init__(self):
        self.k = 100
        self.last_grid = OccupancyGrid()
        self.pose_x = 0.0
        self.pose_y = 0.0

        self.shift = 2000
        self.test_points = get_lane_points()

    def update_grid(self, map_msg):
        new_grid = OccupancyGrid()
        # header
        new_grid.header.seq = 2
        new_grid.header.stamp = rospy.Time.now()
        new_grid.header.frame_id = "map" # CHANGE HERE: odom/map
        # resolution
        new_grid.info = (map_msg.info)
        
        new_grid.data = list((0,)*4000*4000)
        print('got here')

        # i = self.get_array_idx()
        # assert i<4000*4000-1, "i is out of bounds"
        # new_grid.data[i] = 100
        # print('got i: ', i)
        # for j in range(300):
        #     new_grid.data[i+j] = 100
        #     new_grid.data[i-j] = 100

        xcoords = self.test_points[0]/20
        ycoords = self.test_points[1]/20
        for idx, xcoord in enumerate(xcoords):

            x = int(xcoord)
            y = int(ycoords[idx])
            rows = y + 2000
            cols = x + 2000
            i = rows*4000 + cols
            new_grid.data[i] = 100              

        print('exited loop')
        new_grid.data = tuple(new_grid.data)
        print('new grid data: ', new_grid.info)
        self.last_grid = new_grid 

    def update_odom(self, odom_msg):
        self.pose_x = odom_msg.pose.pose.position.x
        self.pose_y = odom_msg.pose.pose.position.y
        # print('updating odom data')
        # print('pose_x: ', self.pose_x)
        # print('pose_y: ', self.pose_y)

    def get_tf_x(self):
        return int(self.pose_x) + self.shift

    def get_tf_y(self):
        return int(self.pose_y) + self.shift

    def get_array_idx(self):
        num_rows = 4000-self.get_tf_y()
        col = self.get_tf_x()
        return 4000*num_rows + col
        



pub = rospy.Publisher('/lane_map', OccupancyGrid, queue_size=1000)
n_state = Nav_State()

def odom_callback(odom_msg):
    # print('Entered the odom callback: ', odom_msg)
    n_state.update_odom(odom_msg)
    # rospy.sleep(1)

def ogrid_callback(ogrid_msg):
    n_state.update_grid(ogrid_msg)
    pub.publish(n_state.last_grid)

def map_listener():
    rospy.init_node("lane_detection")
    ogrid_sub = rospy.Subscriber('/map', OccupancyGrid, ogrid_callback)
    odom_sub  = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    map_listener()
