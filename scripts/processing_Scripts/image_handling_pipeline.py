import os
import shutil
import random

# Paths to the folders
image_folder = "/home/swarm7/Documents/3d_person/sorted_images_200"  # Path to the folder containing the images
label_folder = "/media/swarm7/66D664CC351A1342/Srijan/train_yolo2/target object detection_annotations_2024_12_02_23/obj_train_data"  # Path to the folder containing the labels
data_dir = "/media/swarm7/66D664CC351A1342/Srijan/train_yolo2/data"
output_image_train = data_dir + "images/train"
output_image_val = data_dir + "images/val"
output_label_train = data_dir + "labels/train"
output_label_val = data_dir + "labels/val"

# Create output directories if they don't exist
os.makedirs(output_image_train, exist_ok=True)
os.makedirs(output_image_val, exist_ok=True)
os.makedirs(output_label_train, exist_ok=True)
os.makedirs(output_label_val, exist_ok=True)

# Get the list of image filenames
image_files = [f for f in os.listdir(image_folder) if f.endswith(".png")]

# Shuffle and split into train and validation datasets
random.shuffle(image_files)
split_index = int(0.8 * len(image_files))  # 80% for training, 20% for validation

train_files = image_files[:split_index]
val_files = image_files[split_index:]

# Copy images and labels to their respective folders
for file_list, img_dest, lbl_dest in [
    (train_files, output_image_train, output_label_train),
    (val_files, output_image_val, output_label_val),
]:
    for img_file in file_list:
        base_name = os.path.splitext(img_file)[0]
        label_file = base_name + ".txt"

        # Copy image file
        src_img = os.path.join(image_folder, img_file)
        dest_img = os.path.join(img_dest, img_file)
        shutil.copy(src_img, dest_img)

        # Copy label file
        src_lbl = os.path.join(label_folder, label_file)
        dest_lbl = os.path.join(lbl_dest, label_file)
        if os.path.exists(src_lbl):  # Ensure the label file exists
            shutil.copy(src_lbl, dest_lbl)
        else:
            print(f"Label file {label_file} not found for image {img_file}")

print("Dataset split and copied successfully!")
