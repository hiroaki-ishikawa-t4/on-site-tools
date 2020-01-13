# -*- coding: utf-8 -*-
import subprocess
import signal
import time
import os
import codecs
import csv
import shutil
import rospy

CSV_TEMP_PATH = '/tmp/waypoint_player_tmp.csv'

class WaypointLoaderController(object):
    def __init__(self):
        self.loader_process = None

    def start(self, load_path, override_enable, override_velocity):
        # type: (str, bool, float) -> bool
        if not os.path.exists(load_path):
            rospy.logerr('[on-site-tools] File is not exists!')
            return False

        if override_enable:
            self.provide_overrided_csv(load_path, override_velocity)
        else:
            shutil.copy(load_path, CSV_TEMP_PATH)

        # Run waypoint saver
        self.loader_process = subprocess.Popen(['rosrun', 'waypoint_maker', 'waypoint_loader', '_multi_lane_csv:=%s' % CSV_TEMP_PATH])

        # Check if finish soon. If so, something must be wrong.
        time.sleep(0.5)
        if self.loader_process.poll() != None:
            self.loader_process = None
            rospy.logerr('[on-site-tools] waypoint_loader exited very soon.')
            return False

        return True

    def stop(self):
        # type: () -> None
        if self.loader_process:
            # If waypoint_loader is running, close and wait.
            try:
                os.killpg(os.getpgid(self.loader_process.pid), signal.SIGINT)
            except OSError:
                rospy.logwarn('[on-site-tools] Error during killing waypoint_loader')
            self.loader_process.wait()
            self.loader_process = None

            # Clean up temp file
            os.remove(CSV_TEMP_PATH)
        else:
            rospy.logwarn('[on-site-tools] waypoint_loader is not running.')
        return True

    def provide_overrided_csv(self, load_path, override_velocity):
        # type: (str, float) -> None
        with open(load_path) as f:
            reader = csv.reader(f)
            lines = [line for line in reader]

        for i in range(1, len(lines)):
            line_split = lines[i]
            if len(line_split) != 6:
                rospy.logerr('[on-site-tools] Input csv is invalid.')
                return False

            # Override velocity
            line_split[4] = override_velocity

        # Save csv
        with open(CSV_TEMP_PATH, 'w') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerows(lines)
        


        