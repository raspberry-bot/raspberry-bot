from keras.applications.resnet50 import ResNet50
from keras.preprocessing import image
from keras.applications.resnet50 import preprocess_input, decode_predictions
from keras.models import Model
import numpy as np
from os import listdir, walk
from os.path import isfile, join
import itertools

def list_all_files_in(path):
    return [join(path, f) for f in listdir(path) if isfile(join(path, f))]

def predict(img_path, model):
    img = image.load_img(img_path, target_size=(224, 224))
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)
    return model.predict(x)

def find_difference(f1, f2):
    return np.linalg.norm(f1-f2)

def find_differences(feature_vectors):
    similar = {}
    keys = [k for k,v in feature_vectors.items()]
    _min = {}
    for k in keys:
        _min[k] = 10000000
    possible_combinations=list(itertools.combinations(keys, 2))
    for k,v in possible_combinations:
       diff=find_difference(feature_vectors[k],feature_vectors[v])
       if(diff < _min[k]):
           _min[k] = diff
           similar[k] = v
           _min[v] = diff
           similar[v] = k
    return similar, diff


def load_image_db():
  image_db = []
  with open('controls.json', 'r') as image_db_f:
    image_db = json.load(image_db_f)

  image_db_indexed = {}
  for img in image_db:
    image_db_indexed[img['image']] = img['control']
  return image_db_indexed


def driver():
    image_db = load_image_db()
    feature_vectors = {}
    model = ResNet50(weights='imagenet')
    for img_path in list_all_files_in("/home/bot/catkin_ws/data/images"):
        feature_vectors[img_path.split('/')[-1]] = predict(img_path, model)[0]
    results, diff = find_differences(feature_vectors)
    for k,v in results.items():
        print((k, image_db[k]['control'], v, image_db[v]['control'], diff))    
    #print('Predicted:', decode_predictions(preds, top=3)[0])

driver()

# Output Result

# images/shoe.jpg is most similar to: images/shoe1.jpg
# images/shoe1.jpg is most similar to: images/shoe.jpg
# images/bikini.jpg is most similar to: images/dress.jpeg
# images/dress.jpeg is most similar to: images/bikini.jpg
# images/bear.jpg is most similar to: images/printer1.jpg
# images/printer1.jpg is most similar to: images/printer2.jpg
# images/coil1.jpeg is most similar to: images/printer1.jpg
# images/printer2.jpg is most similar to: images/printer1.jpg