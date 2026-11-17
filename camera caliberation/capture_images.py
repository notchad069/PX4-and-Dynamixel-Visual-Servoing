import cv2
import os

# Create folder for images
os.makedirs('calibration_images', exist_ok=True)

print("Trying to open webcam...")
cap = cv2.VideoCapture(0)  # Try 0 first

if not cap.isOpened():
    print("Camera 0 failed, trying camera 1...")
    cap = cv2.VideoCapture(1)
    
if not cap.isOpened():
    print("Camera 1 failed, trying camera 2...")
    cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("ERROR: Could not open any camera!")
    exit()

print("✓ Camera opened successfully!")

count = 0

print("=" * 50)
print("INSTRUCTIONS:")
print("1. Display the checkerboard pattern on another screen")
print("2. Point your webcam at it")
print("3. Press SPACE to capture an image")
print("4. Move the screen/camera to a different angle")
print("5. Capture 20-30 images from different positions")
print("6. Press ESC when done")
print("=" * 50)

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    
    # Add text overlay so you know it's working
    cv2.putText(frame, f"Images captured: {count}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, "SPACE=Capture | ESC=Quit", (10, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Show live feed
    cv2.imshow('Calibration Capture', frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == 27:  # ESC key
        print("\nQuitting...")
        break
    elif key == 32:  # SPACE key
        img_name = f"calibration_images/calib_{count:03d}.jpg"
        cv2.imwrite(img_name, frame)
        print(f"✓ Saved {img_name} (Total: {count + 1})")
        count += 1

cap.release()
cv2.destroyAllWindows()
print(f"\n✓ Done! Captured {count} images in 'calibration_images/' folder")
