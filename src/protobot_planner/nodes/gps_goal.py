#!/usr/bin/python3.8
"""
GPS Goal Node
"""
import math
import rospy
import click
import actionlib
import tf

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix

def dms_to_decimal_format(lat, long):
    """
    Converts a GPS coordinate given in degrees, minutes, seconds format to decimal format.
    Parameters
    ----------
    lat
        The given latitude in DMS
    long
        The given longitude in DMS
    Returns
    -------
    lat, long : tuple[float, float]
        The latitude and longitude in degrees
    """

    # Check for degrees, minutes, seconds format and convert to decimal
    if ',' in lat:
        degrees, minutes, seconds = lat.split(',')
        degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
        if lat[0] == '-': # check for negative sign
            minutes = -minutes
            seconds = -seconds
        lat = degrees + minutes/60 + seconds/3600
    if ',' in long:
        degrees, minutes, seconds = long.split(',')
        degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
        if long[0] == '-': # check for negative sign
            minutes = -minutes
            seconds = -seconds
        long = degrees + minutes/60 + seconds/3600

    lat = float(lat)
    long = float(long)
    rospy.loginfo(f'Given GPS goal: lat {lat}, long {long}.')
    return lat, long

def get_origin_lat_long():
    """
    Gets the current lat long coordinates.
    Returns
    -------
    lat, long : tuple[float, float]
        The latitude and longitude
    """
    # Get the lat long coordinates of our map frame's origin which must be
    # publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
    rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
    # origin_pose = rospy.wait_for_message('initialpose', PoseStamped)
    origin_lat = 49.89#origin_pose.pose.position.y
    origin_long = 8.9#origin_pose.pose.position.x
    rospy.loginfo(f'Received origin: lat {origin_lat}, long {origin_long}.')
    return origin_lat, origin_long

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
    """
    Calculates the x and y translation in meters to reach a goal lat long from the origin lat long
    Parameters
    ----------
    origin_lat
        The start coordinate latitude
    origin_long
        The start coordinate longitude
    goal_lat
        The goal coordinate latitude
    goal_long
        The goal coordinate longitude
    Returns
    -------
    x, y : tuple[float, float]
        The x and y translation from the start to goal coordinate
    """
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    # Compute several geodesic calculations between two GPS points
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
    hypotenuse = distance = g['s12'] # access distance
    rospy.loginfo(f"The distance from the origin to the goal is {distance:.3f} m.")
    azimuth = g['azi1']
    rospy.loginfo(f"The azimuth from the origin to the goal is {azimuth:.3f} degrees.")

    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS)
    # by finding side lenghs of a right-angle triangle
    # Convert azimuth to radians
    azimuth = math.radians(azimuth)
    x = math.cos(azimuth) * hypotenuse # adjacent
    y = math.sin(azimuth) * hypotenuse # opposite
    rospy.loginfo(f"The translation from the origin to the goal is (x,y) {x:.3f}, {y:.3f} m.")

    return x, y

class GpsGoal():
    """
    This class generates a move_base goal from the rover's position
    to a target goal..
    """
    # x, y, z, yaw, roll, pitch of the goal pose w.r.t. rover
    goal_pose = {
        "x" : 0,
        "y" : 0,
        "z" : 0,
        "roll" : 0,
        "pitch" : 0,
        "yaw" : 0
    }

    def __init__(self):
        """
        Intializes gps_goal node and connects itself to move_base.
        """
        rospy.init_node('gps_goal')
        rospy.loginfo("Connecting to move_base...")
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("Connected.")
        rospy.Subscriber('localization/filtered/global_pose', PoseStamped,
                        self.gps_goal_pose_callback)
        rospy.Subscriber('gps/fix', NavSatFix, self.gps_goal_fix_callback)

        # Get the lat long coordinates of our map frame's origin which must be publshed on
        # topic /local_xy_origin. We use this to calculate our goal within the map frame.
        self.origin_lat, self.origin_long = get_origin_lat_long()
    
    def do_gps_goal(self, goal_lat, goal_long):
        """
        Gets the x and y translation to the goal lat long and sets it to move_base to be published.
        Parameters
        ----------
        goal_lat
            The goal coordinate latitude
        goal_long
            The goal coordinate longitude
        """
        #Calculates goal x and y in the frame_id given the frame's origin GPS and a goal GPS location.
        x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
        self.goal_pose["x"] = x
        self.goal_pose["y"] = y

        # Create move_base goal
        self.publish_goal()

    def gps_goal_pose_callback(self, data):
        """
        Subscriber on localization/filtered/global_pose topic.
        Parameters
        ----------
        data
            The data being received
        """
        lat = data.pose.position.y
        long = data.pose.position.x
        self.goal_pose["z"] = data.pose.position.z
        euler = tf.transformations.euler_from_quaternion(data.pose.orientation)
        self.goal_pose["roll"] = euler[0]
        self.goal_pose["pitch"] = euler[1]
        self.goal_pose["yaw"] = euler[2]
        self.do_gps_goal(lat, long)

    def gps_goal_fix_callback(self, data):
        """
        Subcriber on gps/fix topic.
        Parameters
        ----------
        data
            The data being received
        """
        self.do_gps_goal(data.latitude, data.longitude)

    def publish_goal(self):
        """
        Creates a new move_base goal, sends the goal, and waits for goal result.
        """
        x = self.goal_pose["x"]
        y = self.goal_pose["y"]
        z = self.goal_pose["z"]
        roll = self.goal_pose["roll"]
        pitch = self.goal_pose["pitch"]
        yaw = self.goal_pose["yaw"]

        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param('~frame_id','map')
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        rospy.loginfo(f'Executing move_base goal to position (x,y) {x}, {y}, with {yaw} degrees yaw.')
        rospy.loginfo(
            "To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        # Send goal
        self.move_base.send_goal(goal)
        rospy.loginfo(f'Inital goal status: {GoalStatus.to_string(self.move_base.get_state())}')
        status = self.move_base.get_goal_status_text()
        if status:
            rospy.loginfo(status)

        # Wait for goal result
        self.move_base.wait_for_result()
        rospy.loginfo(f'Final goal status: {GoalStatus.to_string(self.move_base.get_state())}')
        status = self.move_base.get_goal_status_text()
        if status:
            rospy.loginfo(status)

@click.command()
@click.option('--lat', prompt='Latitude', help='Latitude')
@click.option('--long', prompt='Longitude', help='Longitude')
@click.option('--roll', '-r', help='Set target roll for goal', default=0.0)
@click.option('--pitch', '-p', help='Set target pitch for goal', default=0.0)
@click.option('--yaw', '-y', help='Set target yaw for goal', default=0.0)
def cli_main(lat, long):
    """
    Send a goal to move_base given latitude and longitude.
    Notes
    -----
    Accepts two GPS formats:
        Decimal format
            gps_goal.py --lat 43.658 --long -79.379
        Degrees, minutes, seconds format
            gps_goal.py --lat 43,39,31 --long -79,22,45
    Parameters
    ----------
    lat
        The goal coordinate latitude
    long
        The goal coordinate longitude
    """
    gps_goal = GpsGoal()

    # Check for degrees, minutes, seconds format and convert to decimal
    lat, long = dms_to_decimal_format(lat, long)
    gps_goal.do_gps_goal(lat, long)

def ros_main():
    #gpsGoal = GpsGoal()
    rospy.spin()

if __name__ == '__main__':
    lat, long = get_origin_lat_long()
    cli_main(lat, long)
