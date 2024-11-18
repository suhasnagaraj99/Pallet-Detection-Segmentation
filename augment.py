from PIL import Image, ImageEnhance
import numpy as np
import os
import shutil

base_path = '/home/suhas99/Suhas/seg/dataset/'
output_train_images = os.path.join(base_path, 'train/images/')
output_train_labels = os.path.join(base_path, 'train/labels/')

image_files = sorted([f for f in os.listdir(output_train_images) if f.endswith('.jpg')])
label_files = sorted([f for f in os.listdir(output_train_labels) if f.endswith('.txt')])

data = list(zip(image_files, label_files))

for img, lbl in data:
    image = Image.open(os.path.join(output_train_images, img))

    enhancer = ImageEnhance.Brightness(image)
    image = enhancer.enhance(np.random.uniform(0.5, 1.5))

    image = np.array(image)

    Image.fromarray(image).save(os.path.join(output_train_images, 'aug_' + img))

    shutil.copy(os.path.join(output_train_labels, lbl), os.path.join(output_train_labels, 'aug_' + lbl))

print("Augmentation complete!")
