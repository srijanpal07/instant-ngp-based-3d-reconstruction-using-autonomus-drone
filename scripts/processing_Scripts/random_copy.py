import os
import random
import shutil

def select_and_copy_images(source_dir, target_dir, num_images=200):
    # Ensure the source directory exists
    if not os.path.exists(source_dir):
        print(f"Source directory '{source_dir}' does not exist.")
        return

    # Create the target directory if it doesn't exist
    os.makedirs(target_dir, exist_ok=True)

    # Get a list of all image files in the source directory
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff')
    all_images = [file for file in os.listdir(source_dir) if file.lower().endswith(image_extensions)]

    # Check if there are enough images in the source directory
    if len(all_images) < num_images:
        print(f"Not enough images in source directory. Found {len(all_images)} images, but need {num_images}.")
        return

    # Randomly select the specified number of images
    selected_images = random.sample(all_images, num_images)

    # Copy selected images to the target directory
    for image in selected_images:
        src_path = os.path.join(source_dir, image)
        dest_path = os.path.join(target_dir, image)
        shutil.copy(src_path, dest_path)

    print(f"Successfully copied {num_images} images to '{target_dir}'.")

# Example usage
source_folder = '/home/swarm7/Documents/3d_person/1969-12-31_run05_usbcam'
target_folder = '/home/swarm7/Documents/3d_person/sorted_images_200'
select_and_copy_images(source_folder, target_folder, num_images=200)
