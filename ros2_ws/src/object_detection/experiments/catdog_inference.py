import os
import numpy as np
import keras
from keras import layers
from tensorflow import data as tf_data
import matplotlib.pyplot as plt
import tensorflow as tf
import logging
from tensorflow.python.client import device_lib

image_size = (180, 180)

img = keras.utils.load_img("dog_test2.webp", target_size=image_size)
plt.imshow(img)
plt.show()

from keras.models import load_model

model = load_model("save_at_4.keras")
img_array = keras.utils.img_to_array(img)
img_array = keras.ops.expand_dims(img_array, 0)  # Create batch axis

predictions = model.predict(img_array)
score = float(keras.ops.sigmoid(predictions[0][0]))
print(f"This image is {100 * (1 - score):.2f}% cat and {100 * score:.2f}% dog.")
