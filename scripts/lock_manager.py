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

import os


class LockManager(object):
    def __init__(self, lock_path):
        self.lock_path = lock_path

    def release_lock(self):
        # type: () -> None
        # Delete lock file
        try:
            os.remove(self.lock_path)
        except OSError:
            print('[on-site-tools] Lock file is missing.')

    def get_lock(self):
        # type: () -> bool

        # Check previous lock
        if os.path.exists(self.lock_path):
            return False
        else:
            # Create lock file
            with open(self.lock_path, 'w') as f:
                f.write('')
            return True
    
    def get_lock_path(self):
        return self.lock_path