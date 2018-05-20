#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion
from collections import namedtuple
import math
import functools
import Bresenham

MAP_PUBLISH_FREQUENCY = 20
M_PER_CELL = 1.0/20.0
BEAM_EXTEND_CELLS = 10
BEAM_THICKNESS_CELLS = 2
OBJECT_WIDTH_CELLS = 2.0

X_CELLS = 800
Y_CELLS = 800

map_belief = []

move_publisher = None
map_publisher = None
iteration = 1

Cell = namedtuple('Cell', ['x', 'y'])

MovementVel = namedtuple('MovementVel', ['forward', 'ccw_angular'])

class Pose(namedtuple('Pose', ['x', 'y', 'angle'])):
    @property
    def cell(self):
        return position_to_cell((self.x, self.y))

current_pose = Pose(0, 0, 0)

class laserScan():

    @staticmethod
    def beam_end(pose, beam_range, beam_angle_radians):
        a = pose.angle + beam_angle_radians
        return (pose.x + beam_range*math.cos(a), pose.y + beam_range*math.sin(a))

    @staticmethod
    def beam_end_cell(pose, beam_range, beam_angle_radians):
        return position_to_cell(laserScan.beam_end(pose, beam_range, beam_angle_radians))

def new_pose(raw_pose):
    global current_pose

    pose = raw_pose.pose.pose
    current_pose = Pose(x=pose.position.x, y=pose.position.y,
                        angle=quaternion_to_angle(pose.orientation))

def sensor_model(cell, scan_end, scan_max, pose, L0, L_OCCUPIED, L_FREE):
    scan_r = distance(pose.cell, scan_end)
    cell_r = distance(pose.cell, cell)
    
    if cell_r > min(scan_max, scan_r + OBJECT_WIDTH_CELLS/2):
        return L0
    elif scan_r < scan_max and abs(cell_r - scan_r) < OBJECT_WIDTH_CELLS/2:
        return L_OCCUPIED
    else:
        return L_FREE

def cells_between(cell0, cell1):
    return Bresenham.line(cell0, cell1, BEAM_EXTEND_CELLS, BEAM_THICKNESS_CELLS)

def distance(cell1, cell2):
    return math.sqrt((cell1.x - cell2.x)**2 + (cell1.y - cell2.y)**2)

def quaternion_to_angle(q):
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

def position_to_cell(pos):
    (x, y) = pos
    return Cell(x=(int(math.floor(x/M_PER_CELL)) + X_CELLS/2),
                y=(int(math.floor(y/M_PER_CELL)) + Y_CELLS/2))

def cell_to_position(cell):
    (x, y) = cell
    return ((x - X_CELLS/2)*M_PER_CELL,
            (y - Y_CELLS/2)*M_PER_CELL)

def publish(pub, msg):
    pub.publish(serialize(msg))

def serialize(msg):
    if (isinstance(msg, MovementVel)):
        return Twist(Vector3(msg.forward, 0, 0),Vector3(0, 0, msg.ccw_angular))
    elif(isinstance(msg, list)):
        grid = OccupancyGrid(header=Header(seq=0, stamp=rospy.Time.now(), frame_id="map"), info=MapMetaData(width=800, height=800, resolution=0.05, map_load_time=rospy.Time.now()))
        serialize_cell = lambda c: int(probability(c)*100)
        for col in range(0, Y_CELLS):
            for row in range(0, X_CELLS):
                grid.data.append(serialize_cell(msg[col][row]))

        return grid

def move(publisher, scan):
    SPEED = 0.2
    
    if scan.ranges[134]<scan.ranges[44]:
        weighted_angular = -0.5
    else:
        weighted_angular = +0.5
    
    publish(publisher, MovementVel(forward = SPEED, ccw_angular = weighted_angular))

def scanned_cells_from(pose, scan_range, scan_angle):
    scan_end_cell = laserScan.beam_end_cell(pose, scan_range, scan_angle)

    return (scan_end_cell, cells_between(pose.cell, scan_end_cell))

def scanned_cells(scan, pose):
    laser_angles = [math.pi/2, math.pi/4, 0, -math.pi/4, -math.pi/2]
    get_cells = functools.partial(scanned_cells_from, pose)
    return [ get_cells(scan.ranges[179], laser_angles[0]),
             get_cells(scan.ranges[134], laser_angles[1]),
             get_cells(scan.ranges[89], laser_angles[2]),
             get_cells(scan.ranges[44], laser_angles[3]),
             get_cells(scan.ranges[0], laser_angles[4]) ]

def new_scan(raw_scan):
    global move_publisher, map_publisher, map_belief, current_pose, iteration

    L_OCCUPIED = 1.0
    L_FREE = 0.0
    L0 = 0.5

    ranges = raw_scan.ranges

    for (end_cell, cells) in scanned_cells(raw_scan, current_pose):
        for (x, y) in cells:
            prev_cell_belief = map_belief[y][x] 
            sensor = sensor_model(Cell(x, y), end_cell, raw_scan.range_max/M_PER_CELL, current_pose, L0, L_OCCUPIED, L_FREE)
            map_belief[y][x] = prev_cell_belief + sensor - L0

    if raw_scan.ranges[134]<raw_scan.ranges[44]:
        weighted_angular = -0.5
    else:
        weighted_angular = +0.5
    publish(move_publisher, MovementVel(forward = 0.2, ccw_angular = weighted_angular))

    iteration = (iteration + 1)%MAP_PUBLISH_FREQUENCY
    if iteration == 0:
        publish(map_publisher, map_belief)

def log_odds(p):
    return math.log(float(p)/(1 - p))

def probability(log_odd):
    return 1 - 1.0/(1 + math.exp(float(log_odd)))
    
def main():
    global move_publisher, map_publisher
    
    rospy.init_node('mapper', anonymous=True)
    
    for y in range(0, Y_CELLS):
        map_belief.append(X_CELLS*[log_odds(0.5)])

    move_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, new_scan, queue_size=1)
    rospy.Subscriber('base_pose_ground_truth', Odometry, new_pose, queue_size=1)
    try:
        rospy.delete_param('/use_sim_time') 
    except:
        print("No parameter to delete")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
