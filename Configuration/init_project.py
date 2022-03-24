#!/usr/bin/env python3

from copy import copy
import os
import shutil
import sys


def copy_file(current , new):
    remove_file(new + current)
    shutil.copy(current , new)
    
def copy_dir(main_dir , current , new):
    if( os.path.exists(new + "/" + current) ):
        shutil.rmtree(new + "/" + current)
        
    shutil.copytree(main_dir + current , new + "/" + current)
    
def remove_file(file):
    if( os.path.exists(file) ):
        os.remove(file)
        
def copy_all(dir , dst):
    files = os.listdir(dir)
    
    for i in files:
        if( os.path.isdir( dir + i) ):
            copy_dir(dir , i , dst)
        else:
            copy_file(dir + i , dst)

if __name__ == '__main__':
    if( len(sys.argv) == 2 ):
        px4_path = sys.argv[1]
        
        sitl_path_destination = px4_path + "Tools/sitl_gazebo/"
        world_path_destination = sitl_path_destination + "worlds/"
        launch_path_destination = px4_path + "launch/"
        models_path_destination = sitl_path_destination + "models/"
        iris_path_destination = models_path_destination + "iris/"
        
        world_path_current = "worlds/"
        launch_path_current = "launch/"
        models_path_current = "models/"
        
        copy_all(launch_path_current , launch_path_destination)
        
        copy_file(models_path_current + "iris.sdf" , iris_path_destination)
        copy_file(models_path_current + "iris_base.xacro" , models_path_destination + "rotors_description/urdf/")
        copy_dir(models_path_current , "plate" , models_path_destination)
        
        copy_all(world_path_current , world_path_destination)
    else:
        print("One argument required ! The argument must be the path to the PX4-Autopilot folder")
    
    
    