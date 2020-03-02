#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from lane_pipeline import get_lane_points
import tf2_ros, geometry_msgs.msg, tf
from tf import TransformListener
from  geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix

## To Do
'''
'''

# Define the class that will store global state of the robot
class NavState:
    def __init__(self):

        self.ogridPub = rospy.Publisher(
            '/lane_map', 
            OccupancyGrid, 
            queue_size=1000
        )
        self.listener     = TransformListener()
        self.test_points  = get_lane_points  ()
        self.odom_counter = 0
        self.map_received = False

        self.latest_ogrid   = OccupancyGrid()
        self.ogridToPublish = OccupancyGrid()
        self.list = list((0,)*4000*4000)

        # Initialize the grid to be painted and published
        self.ogridToPublish.header.stamp = rospy.Time.now()
        self.ogridToPublish.header.frame_id = "odom"

        # Define baselink to map transform based on
        # latest odom data
        self.a = 0
        self.b = 0
        self.d = 0
        self.e = 0
        self.positionX = 0
        self.positionY = 0
    
    def get_lane_coords(self):
        xcoords = self.test_points[0]/10
        ycoords = self.test_points[1]/10
        return xcoords, ycoords

    def publish_new_grid(self):
        print('Start')

        # Paint the lanes onto grid
        xcoords, ycoords = self.get_lane_coords()
        for idx, xcoord in enumerate(xcoords):
            # Normalize coords to unit square at 
            # bottom corner of grid
            x = ((xcoord)       / 1280 * 1000) - 50
            y = ((ycoords[idx]) / 720  * 1000) / 4 + 20

            # Orient lanes parallel to rover
            x_new = (self.a*x - self.b*y)  + 2000
            y_new = (-self.d*x + self.e*y) + 2000

            # Shift lanes to current location of rover
            cols = (x_new + (self.positionX / self.latest_ogrid.info.resolution))
            rows = (y_new + (self.positionY / self.latest_ogrid.info.resolution))

            i = int(cols)*4000 + int(rows)
            self.list[i] = 100

        print('About to Publish')
        self.ogridToPublish.data = tuple(self.list)
        self.ogridPub.publish(self.ogridToPublish)
        print('End')
        n_state.map_received = True

    def update_odom(self, odom_msg):
        t = self.listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.listener.lookupTransform("/map", "/base_link", t)
        matrix = quaternion_matrix(quaternion)

        self.a = matrix[0][0]
        self.b = matrix[0][1]
        self.d = matrix[1][0]
        self.e = matrix[1][1]
        self.positionY = position[0]
        self.positionX = position[1]


rospy.init_node("lane_detection")
n_state = NavState()

def odom_callback(odom_msg):
    n_state.update_odom(odom_msg)
    n_state.odom_counter += 1
    if(n_state.odom_counter%10 == 0 and n_state.map_received):
        print('-----------About to invoke ogrid publisher')
        n_state.publish_new_grid()    

def ogrid_callback(ogrid_msg):
    print('-----------Updating the map...')
    # n_state.map_received = False
    n_state.latest_ogrid   = ogrid_msg
    for j in range(4000*4000):
        n_state.list[j-1] = n_state.latest_ogrid.data[j-1]
    if(n_state.map_received == False):
        n_state.ogridToPublish.info = ogrid_msg.info
        n_state.publish_new_grid()

def map_listener():
    ogrid_sub = rospy.Subscriber('/map' , OccupancyGrid, ogrid_callback)
    odom_sub  = rospy.Subscriber('/odom', Odometry     , odom_callback )
    rospy.spin()

if __name__ == '__main__':
    map_listener()
