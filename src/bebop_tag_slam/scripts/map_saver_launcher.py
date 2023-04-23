#!/usr/bin/env python3

import os
import sys
import rospkg
import subprocess
from datetime import datetime

def main():
    dt = datetime.now()
    run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('bebop_tag_slam')
    output_folder = os.path.join(package_path, 'tags_and_maps')
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    output_path = os.path.join(output_folder, f'map_{run_id}')
    subprocess.call(['rosrun', 'map_server', 'map_saver', '-f', output_path])

if __name__ == '__main__':
    main()
