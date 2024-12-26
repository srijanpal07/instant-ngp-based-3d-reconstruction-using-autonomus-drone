import cv2
import os

def extract_frames(video_path, output_dir):
    # Ensure the output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)  # Get the original FPS
    frame_count = 0

    print(f"Video FPS: {fps}")
    
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Save the frame as an image
        output_file = os.path.join(output_dir, f"frame_{frame_count:05d}.png")
        cv2.imwrite(output_file, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])  # 0 ensures no compression
        frame_count += 1

    cap.release()
    print(f"Extracted {frame_count} frames to {output_dir}.")

# Parameters
video_file = "/media/swarm7/66D664CC351A1342/Srijan/Final/final_cut.mp4"  # Replace with your .mp4 file path
output_folder = "/media/swarm7/66D664CC351A1342/Srijan/Final/gopro_images"  # Replace with your desired output directory

# Extract frames
extract_frames(video_file, output_folder)

