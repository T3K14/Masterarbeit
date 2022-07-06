from ast import Name
from unicodedata import name
from .auswertung_ma import Read_HO
import argparse

'''
Skript, das mich Datenauswertung auf dem Cluster durchfuehren laesst

'''

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("path_HO", help="The path to the main folder (Hauptordner)")
    parser.add_argument("--read_tracking", help="Bool if tracking files should be read in", type=float, default=1.0)

    
    args  = parser.parse_args()



    for ho in os.listdir(path_HO):
#     print(ho)
    n = int(ho.split("_")[1])
    data1[n] = auswertung_ma.Read_HO(os.path.join(p1, ho), 'p', -2, read_tracking=False, read_lp=False)