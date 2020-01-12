#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from lane_pipeline import get_lane_points
import tf2_ros, geometry_msgs.msg, tf
from tf import TransformListener
from  geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix



# Define the class that will store global state of the robot
class Nav_State:
    def __init__(self):
        self.k = 100
        self.last_grid = OccupancyGrid()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.shift = 2000
        self.listener = TransformListener()
        self.test_points = get_lane_points()


        self.a = 0
        self.b = 0
        self.d = 0
        self.e = 0

        self.position_x = 0
        self.position_y = 0

    def update_grid(self, map_msg):
        new_grid = OccupancyGrid()


        # header
        new_grid.header.seq = 2
        new_grid.header.stamp = rospy.Time.now()
        new_grid.header.frame_id = "odom" # CHANGE HERE: odom/map
        # resolution
        new_grid.info = (map_msg.info)
        print(new_grid.info)
        
        new_grid.data = list((0,)*4000*4000)
        # print('got here')

        # i = self.get_array_idx()
        # assert i<4000*4000-1, "i is out of bounds"
        # new_grid.data[i] = 100
        # print('got i: ', i)
        # for j in range(300):
        #     new_grid.data[i+j] = 100
        #     new_grid.data[i-j] = 100
        for j in range(4000*4000):
            if(map_msg.data[j-1] == 100):
                new_grid.data[j-1] = map_msg.data[j-1]

        xcoords = self.test_points[0]/10
        ycoords = self.test_points[1]/10
        for idx, xcoord in enumerate(xcoords):

            x = ((xcoord)       / 1280 * 1000) - 50
            y = ((ycoords[idx]) / 720  * 1000)

            x_new = (self.a*x - self.b*y)  + 2000
            y_new = (-self.d*x + self.e*y) + 2000

            # x_new = int(x) + 2000 
            # y_new = int(y) + 2000 

            # origin_x = int(((self.pose_x - map_msg.info.origin.position.x) / map_msg.info.resolution))
            # origin_y = int(((self.pose_y - map_msg.info.origin.position.y) / map_msg.info.resolution))

            rows = int(y_new + (self.position_y   / map_msg.info.resolution))
            cols = int(x_new + (self.position_x   / map_msg.info.resolution))

            i = cols*4000 + rows
            new_grid.data[i] = 100              

        # print('exited loop')
        new_grid.data = tuple(new_grid.data)
        # print('new grid data: ', new_grid.info)
        self.last_grid = new_grid 

    def update_odom(self, odom_msg):
        self.pose_x = odom_msg.pose.pose.position.x
        self.pose_y = odom_msg.pose.pose.position.y
        # print(odom_msg)
        # if self.listener.frameExists("/map"):
        t = self.listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.listener.lookupTransform("/map", "/base_link", t)
        # print('BASELINK TRANSFORM', position, quaternion)
        matrix = quaternion_matrix(quaternion)
        # print('matrix: ', matrix)

        self.a = matrix[0][0]
        self.b = matrix[0][1]
        self.d = matrix[1][0]
        self.e = matrix[1][1]

        self.position_y = position[0]
        self.position_x = position[1]


    def get_tf_x(self):
        return int(self.pose_x) + self.shift

    def get_tf_y(self):
        return int(self.pose_y) + slf.shift

    def get_array_idx(self):
        num_rows = 4000-self.get_tf_y()
        col = self.get_tf_x()
        return 4000*num_rows + col
        


rospy.init_node("lane_detection")
pub = rospy.Publisher('/lane_map', OccupancyGrid, queue_size=1000)
n_state = Nav_State()

def odom_callback(odom_msg):
    n_state.update_odom(odom_msg)

def ogrid_callback(ogrid_msg):
    n_state.update_grid(ogrid_msg)
    pub.publish(n_state.last_grid)

def map_listener():
    # rospy.init_node("lane_detection")
    # listener = TransformListener()

    # # transform = geometry_msgs.msg.TransformStamped()
    # transform = listener.lookupTransform("odom","map",rospy.Time.now() )
    # print('transform', transform)

    ogrid_sub = rospy.Subscriber('/map', OccupancyGrid, ogrid_callback)
    odom_sub  = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    map_listener()
