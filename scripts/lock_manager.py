# -*- coding: utf-8 -*-
import os

LOCK_PATH = '/tmp/on-site-tool.lock'

class LockManager(object):
    @classmethod
    def release_lock(cls):
        # type: () -> None
        # Delete lock file
        try:
            os.remove(LOCK_PATH)
        except OSError:
            print('[on-site-tools] Lock file is missing.')

    @classmethod
    def get_lock(cls):
        # type: () -> bool

        # Check previous lock
        if os.path.exists(LOCK_PATH):
            return False
        else:
            # Create lock file
            with open(LOCK_PATH, 'w') as f:
                f.write('')
            return True
    
    @classmethod
    def get_lock_path(cls):
        return LOCK_PATH