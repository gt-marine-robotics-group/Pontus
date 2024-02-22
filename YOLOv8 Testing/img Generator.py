import os
import random
import requests
from PIL import Image

# Function to download an image from a URL
def download_image(url, filename):
    response = requests.get(url)
    if response.status_code == 200:
        with open(filename, 'wb') as f:
            f.write(response.content)

# Function to overlay PNG image onto a background image
def overlay_images(background_path, overlay_path, output_path):
    background = Image.open(background_path)
    overlay = Image.open(overlay_path)

    # Resize overlay image to fit within background's dimensions while maintaining aspect ratio
    overlay.thumbnail(background.size, Image.ANTIALIAS)

    # Check if overlay dimensions exceed background dimensions after resizing
    if overlay.size[0] >= background.size[0] or overlay.size[1] >= background.size[1]:
        print(f"Overlay image '{overlay_path}' is too large for background '{background_path}'. Skipping overlay operation.")
        return

    # Randomly rotate overlay image
    rotation_angle = random.randint(0, 360)
    overlay = overlay.rotate(rotation_angle, expand=True)

    # Randomly position overlay on background
    x = random.randint(0, background.width - overlay.width)
    y = random.randint(0, background.height - overlay.height)
    background.paste(overlay, (x, y), overlay)

    # Save the result
    background.save(output_path)

# Define URLs for background images
background_urls = [
    'https://www.fluidra.com/projects//web/app/uploads/2023/04/The-importance-of-pool-depth.jpg',
    'https://media.istockphoto.com/id/184095196/photo/swim.jpg?s=612x612&w=0&k=20&c=GMgHQ3nnrpd3rCf3TwyMuhgsx4XiPvNUYQ8fBEvauBs=',
    # Add more URLs as needed
]
# Define path for overlay PNG image
overlay_filename = 'overlay.png'  # Assuming overlay.png is in the same folder as the script

# Create folders to save images
os.makedirs('output_images', exist_ok=True)

# Process each background image
for i, url in enumerate(background_urls):
    background_filename = f'background_{i + 1}.jpg'
    download_image(url, background_filename)
    
    # Create 15 random images with overlay PNG
    for j in range(15):
        output_folder = f'output_images/folder_{i + 1}'
        os.makedirs(output_folder, exist_ok=True)
        output_path = os.path.join(output_folder, f'result_{j + 1}.jpg')
        overlay_images(background_filename, overlay_filename, output_path)
        print(f'Processed image {i + 1}-{j + 1}')

print('All images processed and saved.')