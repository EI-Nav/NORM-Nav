#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Video utilities for GroundedSAM2 tracker.

This module provides utilities for video creation and processing.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""
# Standard library
import os

# Third-party libraries
import cv2
from tqdm import tqdm


def create_video_from_images(
    image_folder: str, 
    output_video_path: str, 
    frame_rate: int = 25
) -> None:
    """Create a video from a sequence of images.
    
    Args:
        image_folder: Path to folder containing images.
        output_video_path: Path for the output video file.
        frame_rate: Frame rate for the output video.
    """
    # define valid extension
    valid_extensions = [".jpg", ".jpeg", ".JPG", ".JPEG", ".png", ".PNG"]
    
    # get all image files in the folder
    image_files = [f for f in os.listdir(image_folder) 
                   if os.path.splitext(f)[1] in valid_extensions]
    image_files.sort()  # sort the files in alphabetical order
    print(image_files)
    if not image_files:
        raise ValueError("No valid image files found in the specified folder.")
    
    # load the first image to get the dimensions of the video
    first_image_path = os.path.join(image_folder, image_files[0])
    first_image = cv2.imread(first_image_path)
    height, width, _ = first_image.shape
    
    # create a video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # codec for saving the video
    video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (width, height))
    
    # write each image to the video
    for image_file in tqdm(image_files):
        image_path = os.path.join(image_folder, image_file)
        image = cv2.imread(image_path)
        video_writer.write(image)
    
    # source release
    video_writer.release()
    print(f"Video saved at {output_video_path}")

