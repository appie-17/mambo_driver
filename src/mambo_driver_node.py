#!/usr/bin/env python
import sys, os
### Needed to import correct OpenCV version....
sys.path.insert(0,os.environ['HOME'] + '/.local/lib/python2.7/site-packages')

import rospy
import cv2

from threading import Lock
from dynamic_reconfigure.server import Server
from rospkg import RosPack

from cv_bridge import CvBridge, CvBridgeError
import camera_info_manager as cim

from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

from mambo_driver.msg import MamboTelem
from mambo_driver.cfg import MamboConfig

from pyparrot.utils.colorPrint import color_print
from pyparrot.networking.bleConnection import BLEConnection
from pyparrot.Minidrone import Mambo, MinidroneSensors


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)

class MamboNode(Mambo, object):
    def __init__(self):
        self.vid_stream = None

        # Fetch parameters
        self.use_wifi = rospy.get_param('~use_wifi', True)
        self.bluetooth_addr = rospy.get_param('~bluetooth_addr', '')
        self.num_connect_retries = rospy.get_param('~num_connect_retries', 3)
        self.device_node = rospy.get_param('~device_node', "/dev/video1")
        self.cam_fps = rospy.get_param('~cam_fps', 40)      
        default_calib_path = RosPack().get_path('mambo_driver') + '/cam_calib/default.yaml'
        self.calib_path = rospy.get_param('~camera_calib', default_calib_path) 
        self.caminfo = cim.loadCalibrationFile(self.calib_path, 'camera_front')
        self.caminfo.header.frame_id = rospy.get_param('~camera_frame', 'camera_front')

        # Connect to drone
        self.mambo = super(MamboNode, self).__init__(self.bluetooth_addr, self.use_wifi)
        self.sensors = MamboTelem()
        self.odom_msg = Odometry()
        self.odom_msg.pose.pose.orientation.w = 0 ### DELETE?

        rospy.loginfo('Connecting to Mambo drone...')
        connected = self.connect(self.num_connect_retries)
        if not connected:
            rospy.logerr('Failed to connect to Mambo drone')
            raise IOError('Failed to connect to Mambo drone')

        rospy.loginfo('Connected to Mambo drone')
        self.cb_toggle_cam(Empty)
        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        self.pub_telem = rospy.Publisher('telemetry', MamboTelem, queue_size=1, latch=True)
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=1, latch=True)
        self.pub_caminfo = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)    			       
        self.pub_image = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.sub_takeoff = rospy.Subscriber('takeoff', Empty, self.cb_takeoff, queue_size=1)
        self.sub_land = rospy.Subscriber('land', Empty, self.cb_land, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel, queue_size=1)
        self.sub_emergency = rospy.Subscriber('emergency', Empty, self.cb_emergency, queue_size=1)
        self.sub_flattrim = rospy.Subscriber('flattrim', Empty, self.cb_flattrim, queue_size=1)
        self.sub_pilotmode = rospy.Subscriber('pilot_mode', Empty, self.cb_pilot_mode)
        self.sub_togglecam = rospy.Subscriber('toggle_cam', Empty, self.cb_toggle_cam, queue_size=1)
        self.sub_snapshot = rospy.Subscriber('snapshot', Empty, self.cb_snapshot, queue_size=1)
        self.sub_flip = rospy.Subscriber('flip', UInt8, self.cb_flip, queue_size=1)
        self.sub_auto_takeoff = rospy.Subscriber('auto_takeoff', Empty, self.cb_auto_takeoff, queue_size=1)

        # Setup dynamic reconfigure
        self.cfg = None
        self.srv_dyncfg = Server(MamboConfig, self.cb_dyncfg)

        rospy.on_shutdown(self.cb_shutdown)

        rospy.loginfo('Mambo driver node ready')

    def cb_shutdown(self):
        self.disconnect()

    def cb_sensor_update(self, sensor_list):
        if (sensor_list is not None):
            for sensor in sensor_list:
                (sensor_name, sensor_value, sensor_enum, header_tuple) = sensor

                if (sensor_name is not None):
                    if (sensor_name, "enum") in sensor_enum:
            		# grab the string value
                        if (sensor_value > len(sensor_enum[(sensor_name, "enum")])):
                            sensor_value = "UNKNOWN_ENUM_VALUE"
                        else:
                            enum_value = sensor_enum[(sensor_name, "enum")][sensor_value]
                            sensor_value = enum_value
                    #self.sensors_changed[sensor_name] = sensor_value

                    ### Telemetry msg
                    if (sensor_name == "BatteryStateChanged_battery_percent"):
                        self.sensors.battery_percent = sensor_value
                    elif (sensor_name == "FlyingStateChanged_state"):
                        self.sensors.flying_state = sensor_value
                    elif (sensor_name == "FlyingModeChanged_mode"):
                        self.sensors.flying_mode = sensor_value
                    elif (sensor_name == "PlaneGearBoxChanged_state"):
                        self.sensors.plane_gear_box = sensor_value
                    elif (sensor_name == "GunState_id"):
                        self.sensors.gun_id = int(sensor_value)
                    elif (sensor_name == "GunState_state"):
                        self.sensors.gun_state = str(sensor_value)
                    elif (sensor_name == "ClawState_id"):
                        self.sensors.claw_id = int(sensor_value)
                    elif (sensor_name == "ClawState_state"):
                        self.sensors.claw_state = str(sensor_value)                                        
                    elif (sensor_name == "PilotingModeChanged_mode"):
                        self.sensors.pilot_mode = sensor_value
    
                    ### Odometry msg
                    ### Height measured in meters.. 
                    elif (sensor_name == "DroneAltitude_altitude"):
                        self.odom_msg.pose.pose.position.z = sensor_value*1000
                    elif (sensor_name == "DroneQuaternion_q_w"):
                        self.odom_msg.pose.pose.orientation.w = sensor_value
                    elif (sensor_name == "DroneQuaternion_q_x"):
                        self.odom_msg.pose.pose.orientation.x = sensor_value
                    elif (sensor_name == "DroneQuaternion_q_y"):
                        self.odom_msg.pose.pose.orientation.y = sensor_value
                    elif (sensor_name == "DroneQuaternion_q_z"):
                        self.odom_msg.pose.pose.orientation.z = sensor_value                    
                    ### Speed in m/s
                    elif (sensor_name == "DroneSpeed_speed_x"):
                        self.odom_msg.twist.twist.linear.y = sensor_value
                    elif (sensor_name == "DroneSpeed_speed_y"):
                        self.odom_msg.twist.twist.linear.x = sensor_value
                    elif (sensor_name == "DroneSpeed_speed_z"):
                        self.odom_msg.twist.twist.linear.z = sensor_value
                    # TODO: currently not using timestamp of packages which is in msec, approx. since connected with drone
                    else:
                        pass
                else:
                    color_print(
                        "data type %d buffer id %d sequence number %d" % (data_type, buffer_id, sequence_number),
                        "WARN")
                    color_print("This sensor is missing (likely because we don't need it)", "WARN")

            self.odom_msg.child_frame_id = 'Mambo'
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.pose.pose.position.x = 0
            self.odom_msg.pose.pose.position.y = 0
            
            self.pub_telem.publish(self.sensors)
            self.pub_odom.publish(self.odom_msg)

    def cb_dyncfg(self, config, level):
        update_all = False
        if self.cfg is None:
            self.cfg = config
            update_all = True
        if update_all or self.cfg.max_vert_speed_mps != config.max_vert_speed_mps:
            self.set_max_vertical_speed(config.max_vert_speed_mps)
        if update_all or self.cfg.max_tilt_deg != config.max_tilt_deg:
            self.set_max_tilt(config.max_tilt_deg)
        if update_all or self.cfg.preferred_pilot_mode != config.preferred_pilot_mode:
            self.set_preferred_pilot_mode(config.preferred_pilot_mode)
        if update_all or self.cfg.banked_turn_mode != config.banked_turn_mode:
            self.set_banked_turn_mode(config.banked_turn_mode)
        # TODO: are there any other configs from pyparrot? from minidrone.xml?
        self.cfg = config
        return self.cfg

    def cb_takeoff(self, msg):
        success = self.safe_takeoff(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Takeoff', success)

    def cb_auto_takeoff(self, msg):
        success = self.turn_on_auto_takeoff()
        if success:
            rospy.loginfo('Drone set to auto-takeoff when pitched')
        else:
            rospy.logwarn('AutoTakeoff command failed')

    def cb_land(self, msg):
        success = self.safe_land(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Land', success)

    def cb_emergency(self, msg):
        success = self.safe_emergency(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Emergency', success)

    def cb_flattrim(self, msg):
        success = self.flat_trim()
        notify_cmd_success('FlatTrim', success)
        
    def cb_pilot_mode(self, msg):
        success = self.toggle_pilot_mode(self.cfg.cmd_timeout_sec)
        notify_cmd_success('TogglePilotMode', success)

    def cb_toggle_cam(self, msg):

        ### True if loadable kernel module (LKM) not loaded
        if (os.system('lsmod | grep v4l2loopback -q')): 
            rospy.logwarn("Please run command: \"sudo modprobe v4l2loopback\"")

        elif self.vid_stream == None:
            bashCommand = "ffmpeg -i rtsp://192.168.99.1/media/stream2 -f v4l2 /dev/video1 > ~/output.log 2>&1 < /dev/null &".format(self.device_node)
            os.system(bashCommand)
            
            self.bridge = CvBridge()
            self.stream = cv2.VideoCapture(self.device_node)

            rospy.loginfo("Piping stream to \'{}\'".format(self.device_node))        

            rospy.Timer(rospy.Duration(1.0/self.cam_fps), self.publish_video)
            self.vid_stream = True
        
    def publish_video(self, event):
        try:
            _, frame = self.stream.read()

            if frame is not None:
                stamp = rospy.Time.now()
            	img_msg = self.bridge.cv2_to_imgmsg(frame,'rgb8')
            	img_msg.header.frame_id = self.caminfo.header.frame_id
                img_msg.header.stamp = stamp
                self.pub_image.publish(img_msg)
                
                self.caminfo.header.stamp = stamp 
                self.pub_caminfo.publish(self.caminfo)

        except CvBridgeError as e:
            print(e)
                
    def cb_snapshot(self, msg):
        success = None
        notify_cmd_success('Snapshot', success)

    def cb_flip(self, msg):
        if msg.data == 1:
            task = 'Flip Front'
            success = self.flip('front')
        elif msg.data == 2:
            task = 'Flip Back'
            success = self.flip('back')
        elif msg.data == 3:
            task = 'Flip Right'
            success = self.flip('right')
        elif msg.data == 4:
            task = 'Flip Left'
            success = self.flip('left')
        else:
            rospy.logwarn('Invalid flip direction: %d' % msg.data)
            return
        notify_cmd_success(task, success)

    def cb_cmd_vel(self, msg):
        pitch = msg.linear.y*100
        roll = msg.linear.x*100
        yaw = msg.angular.z*100
        vertical_movement = msg.linear.z*100
        
        self.fly_direct(roll, pitch, yaw, vertical_movement) # TODO: fix this: don't send if drone state not flying


########################    ADDING FUNCTIONS TO MAMBO INTERFACE   ################################
    def set_banked_turn_mode(self, arg):
        """
        Turn on/off the banked turn mode
        :return: True if the command was sent and False otherwise
        """
        command_tuple = self.command_parser.get_command_tuple("minidrone", "PilotingSettings", "BankedTurn")

        return self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[arg], param_type_tuple=["u8"])

#########################    OVERRIDING FUNCTIONS OF MAMBO INTERFACE   ###########################

    def set_preferred_pilot_mode(self, mode):
        """
        Sets the preferred piloting mode. Ensures you choose from "easy", "medium", "difficult".

        :param value: preferred piloting mode
        :return: True if the command was sent and False otherwise
        """

        if (mode not in ["easy", "medium", "difficult"]):
            print("Ensures you choose piloting mode from \"easy\", \"medium\", \"difficult\".")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("minidrone", "PilotingSettings", "PreferredPilotingMode", mode)

        return self.drone_connection.send_enum_command_packet_ack(command_tuple,enum_tuple)
    
    def toggle_pilot_mode(self, timeout):
        """
        Sends the TogglePilotMode command to the mambo.  Gets the codes for it from the xml files.  Ensures the
        packet was received or sends it again up to a maximum number of times.

        :return: True if the command was sent and False otherwise
        """
        start_time = rospy.Time.now()

        pilot_mode = self.sensors.pilot_mode

        while (self.sensors.pilot_mode == pilot_mode and (rospy.Time.now() - start_time < rospy.Duration(timeout))):
            if (self.sensors.flying_state == "emergency"):
                return
            color_print("changing pilot_mode", "INFO")
            command_tuple = self.command_parser.get_command_tuple("minidrone", "Piloting", "TogglePilotingMode")            
            self.smart_sleep(1)

        return self.drone_connection.send_noparam_command_packet_ack(command_tuple)

    def smart_sleep(self, timeout):
        """
        Re-implemented smart sleep to use rospy.sleep

        Do not call time.sleep directly as it will mess up BLE and miss WIFI
        packets! This function handles packets received while sleeping.

        :param timeout: number of seconds to sleep
        """
        start_time = rospy.Time.now()
        dt = (rospy.Time.now() - start_time).to_sec()
        # DISABLED TO PREVENT ASYMPTOTIC ITERATION: sleep_quanta_sec = min(timeout-dt, self.cfg.sleep_quanta_sec)
        sleep_quanta_sec = self.cfg.sleep_quanta_sec

        while dt < timeout:
            if issubclass(type(self.drone_connection), BLEConnection):
                try:
                    notify = self.drone_connection.drone_connection.waitForNotifications(
                        sleep_quanta_sec)
                except:
                    # color_print("reconnecting to wait", "WARN")
                    self.drone_connection._reconnect(self.num_connect_retries)
            else:  # assume WifiConnection
                rospy.sleep(sleep_quanta_sec)
            dt = (rospy.Time.now() - start_time).to_sec()

	#Modify MinidroneSensors class to process sensor values directly into ROS msg
    def update_sensors(self, data_type, buffer_id, sequence_number, raw_data, ack):
        """
        Update the sensors (called via the wifi or ble connection)

        :param data: raw data packet that needs to be parsed
        :param ack: True if this packet needs to be ack'd and False otherwise
        """
        self.sensors_updated = {}
        self.update_timeLast = rospy.Time.now()
        sensor_list = self.sensor_parser.extract_sensor_values(raw_data)

		#Dictionary with updated sensors from current package send by drone
       
        #print("Sensors changed {},\n secs since last package: {}".format(self.sensors_changed, (rospy.Time.now() - self.update_timeLast).to_sec()))
        self.cb_sensor_update(sensor_list)

        if (ack):
            self.drone_connection.ack_packet(buffer_id, sequence_number)                

#################################################################################

def main():
    rospy.init_node('mambo_node')
    robot = MamboNode()
    rospy.spin()

if __name__ == '__main__':
    main()
