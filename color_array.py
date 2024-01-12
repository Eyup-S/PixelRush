from PIL import Image
import numpy as np

def image_to_string(image_path, output_path):
    img = Image.open(image_path)

    # Optionally resize the image to 240x320
    #img_resized = img.resize((320, 240))
    img_resized = img

    if img_resized.mode != 'RGB':
        img_resized = img_resized.convert('RGB')

    img_array = np.array(img_resized)

    
    array_string = "uint16_t image[height][width] = {\n" # Change width and height
    for row in img_array:
        array_string += "    {"
        for pixel in row:

            r, g, b = pixel
            color_16bit = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            array_string += str(color_16bit) + ", "
        array_string = array_string[:-2] + "},\n"
    array_string = array_string[:-2] + "\n};"

    with open(output_path, 'w') as file:
        file.write(array_string)

if "__name__" == "__main__":
    image_path = "resources/cover.png"  
    output_path = "cover.txt"  

    image_to_string(image_path, output_path)