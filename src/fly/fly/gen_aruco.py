#!/usr/bin/env python3
"""
Generate ArUco marker images for Gazebo simulation
"""

import cv2
import numpy as np
import os

def generate_aruco_marker(marker_id=0, size=1000, output_dir="."):
    """
    Generate an ArUco marker image
    
    Args:
        marker_id: ID of the marker (0-249 for DICT_6X6_250)
        size: Size of the output image in pixels
        output_dir: Directory to save the marker
    """
    # Create ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    
    # Generate marker image
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    
    # Add white border (10% of size on each side)
    border_size = int(size * 0.1)
    marker_with_border = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )
    
    # Save marker
    output_path = os.path.join(output_dir, f"aruco_marker_{marker_id}.png")
    cv2.imwrite(output_path, marker_with_border)
    print(f"Generated ArUco marker {marker_id}: {output_path}")
    print(f"Image size: {marker_with_border.shape[0]}x{marker_with_border.shape[1]} pixels")
    
    return output_path

def generate_multiple_markers(marker_ids=[0, 1, 2, 3], size=1000, output_dir="."):
    """Generate multiple ArUco markers"""
    os.makedirs(output_dir, exist_ok=True)
    
    paths = []
    for marker_id in marker_ids:
        path = generate_aruco_marker(marker_id, size, output_dir)
        paths.append(path)
    
    print(f"\nGenerated {len(paths)} markers in {output_dir}")
    return paths

if __name__ == "__main__":
    # Generate markers with IDs 0-3
    output_directory = "./aruco_markers"
    generate_multiple_markers(
        marker_ids=[0, 1, 2, 3, 4],
        size=1000,
        output_dir=output_directory
    )
    
    print("\n" + "="*60)
    print("To use these markers in Gazebo:")
    print("1. Copy the generated PNG files to your Gazebo resources")
    print("2. Update your SDF file to reference these images")
    print("="*60)
