# importing libraries
import json

class Configuration:
    def __init__(self):
        with open('configure.json') as config:
            self.data = json.load(config)

    def get(self, index):
        try:
            return self.data[index]
        except IndexError:
            return None

