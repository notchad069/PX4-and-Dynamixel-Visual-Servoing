#!/usr/bin/env python3
"""
Generate ArUco marker images and replace PX4's marker
"""

import cv2
import numpy as np
import os
import shutil

def generate_aruco_marker(marker_id=0, size=1000, output_path="aruco_marker.png"):
    """
    Generate an ArUco marker image with white border
    
    Args:
        marker_id: ID of the marker (0-249 for DICT_6X6_250)
        size: Size of the marker pattern in pixels (before border)
        output_path: Where to save the marker
    """
    # Create ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    
    # Generate marker image (black and white pattern)
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    
    # Add white border (20% of size on each side for better detection)
    border_size = int(size * 0.2)
    marker_with_border = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255  # White border
    )
    
    # Save marker
    cv2.imwrite(output_path, marker_with_border)
    print(f"✓ Generated ArUco marker ID {marker_id}")
    print(f"  File: {output_path}")
    print(f"  Size: {marker_with_border.shape[0]}x{marker_with_border.shape[1]} pixels")
    
    return output_path

def replace_px4_marker(marker_id=0):
    """
    Generate and replace PX4's ArUco marker texture
    """
    # Find PX4 directory
    px4_path = os.environ.get('PX4_AUTOPILOT')
    if not px4_path:
        print("ERROR: PX4_AUTOPILOT environment variable not set!")
        print("Please run: export PX4_AUTOPILOT=/path/to/PX4-Autopilot")
        return False
    
    # Path to the marker texture
    marker_texture_path = os.path.join(
        px4_path,
        'Tools/simulation/gz/models/aruco_marker/materials/textures/aruco_marker_0.png'
    )
    
    if not os.path.exists(os.path.dirname(marker_texture_path)):
        print(f"ERROR: Directory not found: {os.path.dirname(marker_texture_path)}")
        return False
    
    # Backup original if it exists
    if os.path.exists(marker_texture_path):
        backup_path = marker_texture_path + '.backup'
        shutil.copy2(marker_texture_path, backup_path)
        print(f"✓ Backed up original to: {backup_path}")
    
    # Generate new marker
    temp_marker = "/tmp/aruco_marker_0_new.png"
    generate_aruco_marker(marker_id=marker_id, size=1000, output_path=temp_marker)
    
    # Copy to PX4 directory
    shutil.copy2(temp_marker, marker_texture_path)
    print(f"✓ Replaced marker texture: {marker_texture_path}")
    print(f"\nNow restart Gazebo to see the new marker!")
    
    return True

def generate_multiple_markers_for_testing(output_dir="./aruco_test_markers"):
    """
    Generate multiple markers for testing
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"\nGenerating test markers in {output_dir}/")
    print("=" * 60)
    
    for marker_id in range(5):
        output_path = os.path.join(output_dir, f"aruco_marker_{marker_id}.png")
        generate_aruco_marker(marker_id=marker_id, size=1000, output_path=output_path)
    
    print("=" * 60)
    print(f"Generated 5 test markers in {output_dir}/")
    print("You can view these to verify they look correct before replacing PX4's marker")

if __name__ == "__main__":
    import sys
    
    print("ArUco Marker Generator for PX4")
    print("=" * 60)
    
    # First, generate test markers to verify
    print("\n1. Generating test markers...")
    generate_multiple_markers_for_testing()
    
    print("\n2. Check the test markers in ./aruco_test_markers/")
    print("   They should show black/white grid patterns, NOT solid black!")
    
    # Ask user if they want to replace PX4's marker
    response = input("\nDo you want to replace PX4's marker now? (yes/no): ").strip().lower()
    
    if response in ['yes', 'y']:
        marker_id = input("Enter marker ID to generate (0-249, default=0): ").strip()
        marker_id = int(marker_id) if marker_id else 0
        
        print(f"\n3. Replacing PX4's marker with ID {marker_id}...")
        if replace_px4_marker(marker_id=marker_id):
            print("\n✓ SUCCESS! Restart Gazebo to see the new marker.")
            print("  Command: make px4_sitl gz_x500_depth")
        else:
            print("\n✗ FAILED! Check the error messages above.")
    else:
        print("\nSkipped PX4 marker replacement.")
        print("To replace manually:")
        print("  1. Check test markers in ./aruco_test_markers/")
        print("  2. Copy aruco_marker_0.png to:")
        print("     $PX4_AUTOPILOT/Tools/simulation/gz/models/aruco_marker/materials/textures/")