# -*- coding: utf-8 -*-
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