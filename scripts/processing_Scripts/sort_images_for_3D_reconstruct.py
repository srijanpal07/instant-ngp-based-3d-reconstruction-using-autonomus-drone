#!/usr/bin/env python3

import cv2
import os
import random
import shutil
from sklearn.metrics import accuracy_score



def calculate_laplacian_variance(image_path):
    """Calculate the Laplacian variance of an image."""
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    return laplacian.var()



def find_optimal_threshold(variances):
    """Automatically find the optimal threshold by maximizing separation between clusters."""
    sorted_variances = sorted(variances)
    max_diff = 0
    optimal_threshold = 0
    
    # Iterate through variance values to find the largest gap
    for i in range(1, len(sorted_variances)):
        diff = sorted_variances[i] - sorted_variances[i - 1]
        if diff > max_diff:
            max_diff = diff
            optimal_threshold = (sorted_variances[i] + sorted_variances[i - 1]) / 2
    return optimal_threshold



def filter_non_blurry_images(source_dir, threshold):
    """Filter non-blurry images based on a threshold."""
    non_blurry_images = []
    for filename in os.listdir(source_dir):
        if filename.endswith(('.png', '.jpg', '.jpeg')):
            image_path = os.path.join(source_dir, filename)
            variance = calculate_laplacian_variance(image_path)
            if variance >= threshold:
                non_blurry_images.append(image_path)
    return non_blurry_images



def select_and_copy_images(images, target_dir, num_to_select=200):
    """Randomly select images and copy them to a new directory."""
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    selected_images = random.sample(images, min(num_to_select, len(images)))
    for image_path in selected_images:
        shutil.copy(image_path, target_dir)



def main():
    source_dir = '/media/swarm7/66D664CC351A1342/Srijan/Final/gopro_images'  # Update with your source directory
    target_dir = '/media/swarm7/66D664CC351A1342/Srijan/Final/go_pro_sorted'  # Update with your target directory

    # Step 1: Calculate Laplacian variances for all images
    variances = []
    image_paths = []
    for filename in os.listdir(source_dir):
        if filename.endswith(('.png', '.jpg', '.jpeg')):
            image_path = os.path.join(source_dir, filename)
            image_paths.append(image_path)
            variance = calculate_laplacian_variance(image_path)
            variances.append(variance)

    # Step 2: Find the optimal threshold
    optimal_threshold = find_optimal_threshold(variances)
    print(f"Optimal Threshold: {optimal_threshold}")
    # optimal_threshold = 1000

    # Step 3: Filter non-blurry images
    non_blurry_images = filter_non_blurry_images(source_dir, optimal_threshold)
    print(f"Number of Non-Blurry Images: {len(non_blurry_images)}")

    # Step 4: Randomly select 200 images and copy them to the target directory
    select_and_copy_images(non_blurry_images, target_dir, num_to_select=400)
    print(f"Copied {min(400, len(non_blurry_images))} images to {target_dir}")



if __name__ == "__main__":
    main()
