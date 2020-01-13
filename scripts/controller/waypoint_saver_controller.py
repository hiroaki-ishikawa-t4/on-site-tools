# -*- coding: utf-8 -*-
import subprocess
import signal
import time
import os
import codecs
import rospy

class WaypointSaverController(object):
    def __init__(self):
        self.saver_process = None

    def start_record(self, save_path):
        # type: (str) -> bool
        if os.path.exists(save_path):
            rospy.logerr('[on-site-tools] File is already exists!')
            return False

        # Run waypoint saver
        self.saver_process = subprocess.Popen(['roslaunch', 'waypoint_maker', 'waypoint_saver.launch', 'save_finename:=%s' % save_path, 'input_type:=0', 'save_velocity:=True', 'interval:=1'])

        # Check if finish soon. If so, something must be wrong.
        time.sleep(0.5)
        if self.saver_process.poll() != None:
            self.saver_process = None
            rospy.logerr('[on-site-tools] waypoint_saver exited very soon.')
            return False

        return True

    def finish_record(self):
        # type: () -> None
        if self.saver_process:
            # If waypoint_saver is running, close and wait.
            try:
                os.killpg(os.getpgid(self.saver_process.pid), signal.SIGINT)
            except OSError:
                rospy.logwarn('[on-site-tools] Error during killing waypoint_saver')
            self.saver_process.wait()
            self.saver_process = None
        else:
            rospy.logwarn('[on-site-tools] waypoint_saver is not running.')
        return True        
