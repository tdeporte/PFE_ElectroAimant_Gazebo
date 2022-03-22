#!/usr/bin/env python3

from copy import copy
import os
import shutil
import sys


def copy_file(current , new):
    remove_file(new + current)
    shutil.copy(current , new)
    
def copy_dir(current , new):
    if(os.path.exists(new + "/" + current)):
        shutil.rmtree(new + "/" + current)
    
    shutil.copytree(current , new + "/" + current)
    
def copy_dir2(main_dir , current , new):
    if( os.path.exists(new + "/" + current) ):
        shutil.rmtree(new + "/" + current)
        
    shutil.copytree(main_dir + current , new + "/" + current)
    
def remove_file(file):
    if( os.path.exists(file) ):
        os.remove(file)

if __name__ == '__main__':
    px4_path = sys.argv[1]
    
    world_path_destination = px4_path + "Tools/sitl_gazebo/worlds/"
    launch_path_destination = px4_path + "launch/"
    models_path_destination = px4_path + "Tools/sitl_gazebo/models/"
    iris_path_destination = models_path_destination + "iris/"
    
    world_path_current = "worlds/"
    launch_path_current = "launch/"
    models_path_current = "models/"
        
    copy_file(launch_path_current + "custom.launch" , launch_path_destination)
    
    copy_file(models_path_current + "iris.sdf" , iris_path_destination)
    copy_file(models_path_current + "iris_base.xacro" , models_path_destination + "rotors_description/urdf/")
    copy_dir2(models_path_current , "plate" , models_path_destination)
    
    copy_file(world_path_current + "PFE.world" , world_path_destination)
    
    dirs = os.listdir('Library/')
    for i in dirs:
        if( os.path.isdir( "Library/" + i ) ):
            copy_dir2("Library/" , i, px4_path + "Tools/sitl_gazebo")
        else:
            copy_file("Library/" + i , px4_path + "Tools/sitl_gazebo")
    