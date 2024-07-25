#!/usr/bin/env python3

"""
script to build fast rate attitude firmware and copy to my dropbox

AP_FLAKE8_CLEAN
"""

import os
import shutil
import subprocess
import sys

os.environ['PYTHONUNBUFFERED'] = '1'


def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        sys.exit(1)

version='v1'

for f in ["Pixhawk6C-bdshot", "Pixhawk6X-bdshot", "CUAVv5Nano-bdshot"]:
    print("Building firmware %s" % f)
    run_program(["./waf", "configure", "--board", f])
    run_program(["./waf", "clean"])
    run_program(["./waf", "copter"])
    os.makedirs('/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s' % (version, f), exist_ok=True)
    shutil.copy('build/%s/bin/arducopter.apj' % f, '/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s' % (version, f))
    shutil.copy('build/%s/bin/arducopter_with_bl.hex' % f, '/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s' % (version, f))

    run_program(["./waf", "configure", "--board", f, "--enable-stats"])
    run_program(["./waf", "clean"])
    run_program(["./waf", "copter"])
    os.makedirs('/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s-stats' % (version, f), exist_ok=True)
    shutil.copy('build/%s/bin/arducopter.apj' % f, '/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s-stats' % (version, f))
    shutil.copy('build/%s/bin/arducopter_with_bl.hex' % f, '/mnt/c/Users/Andy/Dropbox/Public/firmware/fast_rate/%s/%s-stats' % (version, f))
