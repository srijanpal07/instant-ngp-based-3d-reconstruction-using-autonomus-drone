import cv2
import os
import glob

def create_video_from_images(image_dir, output_video_path, fps):
    # Get list of image files sorted in sequential order
    image_files = sorted(glob.glob(os.path.join(image_dir, "*.png")))  # Change to "*.png" if using PNG images
    
    if not image_files:
        print("No images found in the directory!")
        return
    
    # Read the first image to get the dimensions
    first_image = cv2.imread(image_files[0])
    height, width, layers = first_image.shape

    # Define the video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4
    video = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    # Write each image to the video
    for image_file in image_files:
        frame = cv2.imread(image_file)
        video.write(frame)

    video.release()
    print(f"Video created at: {output_video_path}")

# Parameters
image_directory = "/home/swarm7/Documents/3d_person/deploy/detection/2024-12-05_run03_detection"  # Replace with the directory containing images
output_video_file = "/home/swarm7/Documents/3d_person/deploy/detection/output_detection.mp4"  # Replace with your desired output video file path
frames_per_second = 25  # Desired FPS

# Create video
create_video_from_images(image_directory, output_video_file, frames_per_second)

