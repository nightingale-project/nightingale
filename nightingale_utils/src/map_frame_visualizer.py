#!/usr/bin/env python3
import numpy as np
import sys
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import argparse
import os
import yaml

def read_data(path):
    with open(path, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
        res = data_loaded['resolution']
        origin = np.array(data_loaded['origin'][:-1])
        image_name = data_loaded['image']
        dir_name = os.path.dirname(path)
        image_path = os.path.join(dir_name,image_name)
        img = mpimg.imread(image_path)
        return img,res,origin

def main():
    parser = argparse.ArgumentParser(description='Visualize a ROS map. This shows the coordinates of the map frame.')
    parser.add_argument('filename',help='path to map.yaml file')
    args = parser.parse_args()
    assert os.path.exists(args.filename)
    grid,resolution,origin = read_data(args.filename)
    print(np.unique(grid))
    print(f'Image Metadata: shape:{grid.shape} resolution:{resolution} origin:{origin}')
    OCCUPIED = 0
    UNKNOWN = 205
    occupied = np.fliplr(np.argwhere(grid==OCCUPIED)*resolution)
    occupied += origin
    unknown = np.fliplr(np.argwhere(grid==UNKNOWN)*resolution)
    unknown += origin

    figure, axis = plt.subplots(1, 2)
    
    #axis[0].axes().set_aspect('equal')
    axis[0].scatter(occupied[:,0],occupied[:,1],s=0.05,c='red')
    axis[0].scatter(unknown[:,0],unknown[:,1],s=0.05,c='black')
    axis[0].set_title('Map in Map Frame')
    axis[1].imshow(grid)
    axis[1].set_title('Map in Pixel Frame')
    axis[1].invert_yaxis()
    plt.show()

if __name__ == '__main__':
    main()
