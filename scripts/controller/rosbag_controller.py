# -*- coding: utf-8 -*-
import subprocess
import signal
import time
import os
import codecs
import rospy


class RosbagController(object):

    def __init__(self):
        self.rosbag_process = None

    def start_record(self, root_dir, title, description, options):
        # type: (str, str, str, str, str) -> bool
        if self.rosbag_process:
            rospy.logwarn('[on-site-tools] rosbag is already running.')
            return False

        # Provide save directory
        root_dir = (root_dir+'/').replace('//', '/')
        save_dir = root_dir + title
        if os.path.exists(save_dir):
            return False
        else:
            os.mkdir(save_dir)

        # Save description
        with codecs.open(save_dir+'/description.txt', 'w', 'utf-8') as f:
            f.write(description)
            f.close()
        
        # Save information
        with open(save_dir+'/information.txt', 'w') as f:
            f.write(self.collect_information())
            f.close()

        # Start rosbag
        cmd = '/opt/ros/melodic/bin/rosbag record ' + options.replace('\n', ' ')
        rospy.loginfo('[on-site-tools] Will execute ' + cmd)
        self.rosbag_process = subprocess.Popen(cmd, shell=True, cwd=save_dir)

        # Check if finish soon. If so, something must be wrong.
        time.sleep(0.5)
        if self.rosbag_process.poll() != None:
            self.rosbag_process = None
            return False
        
        return True

    def finish_record(self):
        # type: () -> None
        if self.rosbag_process:
            # If rosbag is running, close and wait.
            try:
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
            except OSError:
                rospy.logwarn('[on-site-tools] Error during killing rosbag')
            self.rosbag_process.wait()
            self.rosbag_process = None
        else:
            rospy.logwarn('[on-site-tools] rosbag is not running.')
        return True

    def collect_information(self):
        # type: () -> str
        try:
            date_time = subprocess.check_output(['date'])
            kernel_info = subprocess.check_output(['uname', '-a'])
            topics = subprocess.check_output(['/opt/ros/melodic/bin/rostopic', 'list'])
            result = 'datetime:\n%s\n\nkernel info:\n%s\n\ntopics:\n%s\n\n' % (date_time, kernel_info, topics)
            print(result)
            return result
        except:
            rospy.logwarn('[on-site-tools] Error collecting information. Continue...')
            return ''
