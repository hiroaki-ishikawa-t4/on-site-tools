# -*- coding: utf-8 -*-
#
# Copyright 2020 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess
import signal
import time
import os
import rospy

class CandumpController(object):
    def __init__(self):
        self.candump_process = None

    def check_candump_exist(self):
        try:
            subprocess.check_call(['candump'])
        except (subprocess.CalledProcessError, OSError):
            return False
        return True

    def start(self, device_name, output_dir):
        # type: (str, str) -> bool
        # Start candump
        cmd = 'candump -l %s' % device_name
        rospy.loginfo('[on-site-tools] Will execute ' + cmd)
        self.candump_process = subprocess.Popen(cmd, shell=True, cwd=output_dir, preexec_fn=os.setsid)

        # Check if finish soon. If so, something must be wrong.
        time.sleep(0.5)
        if self.candump_process.poll() != None:
            self.candump_process = None
            return False
        return True

    def finish(self):
        if self.candump_process:
            # If candump is running, close and wait.
            try:
                os.killpg(os.getpgid(self.candump_process.pid), signal.SIGINT)
            except OSError:
                rospy.logwarn('[on-site-tools] Error during killing candump')
            self.candump_process.wait()
            self.candump_process = None
        else:
            rospy.logwarn('[on-site-tools] candump is not running.')
        return True

    def is_working(self):
        # type: () -> bool
        return self.candump_process != None
