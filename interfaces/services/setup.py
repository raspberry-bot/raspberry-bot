import json

class Config:
    def __init__(self, name, payload):
        self.name = name
        self.payload = payload
    
    def append_to_global_config(self):
        with open('../../config/global.json', 'r+') as global_config_f:
            data = json.load(global_config_f)
            data[self.name] = self.payload
            global_config_f.seek(0)
            json.dump(data, global_config_f, indent=4)
            global_config_f.truncate()

