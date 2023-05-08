#!/bin/bash

# Set the desired output video name
output_video_name="output_video.mp4"

# Set the desired frame rate for the video
frame_rate="180"

# Run the ffmpeg command to generate the MP4 video from the input frames
ffmpeg -r $frame_rate -pattern_type glob -i 'frame_*.png' -vcodec libx264 -pix_fmt yuv420p -y $output_video_name