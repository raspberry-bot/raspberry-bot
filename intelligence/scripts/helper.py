import os

from keras.engine.saving import model_from_json


def save_model(project_path, model_name, model):
    print('Saving model...')
    # serialize model to JSON
    model_json = model.to_json()
    with open(os.path.join(project_path, '%s.json' % model_name), "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    model.save_weights(os.path.join(project_path, '%s.h5' % model_name))
    print("Saved model to disk")


def load_model(project_path, model_name):
    # load json and create model
    json_file = open(os.path.join(project_path, '%s.json' % model_name), 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)
    # load weights into new model
    loaded_model.load_weights(os.path.join(project_path, '%s.h5' % model_name))
    print("Loaded model from disk")
    return loaded_model