import sys
import os




COLORADAR_PATH = os.path.join(os.path.expanduser('~'), 'coloradar')
dataset = ct.ColoradarDataset(COLORADAR_PATH)
runs = dataset.getRuns()
print(runs)