from auswertung_ma import Read_HO, rsb_vorauswertung
import argparse
import os

'''
Skript, das mich Vorauswertung der RSB-Daten auf dem Cluster durchfuehren laesst

Ich gehe in jeden Konfig-Ordner im Hauptordner rein und fuehre dort eine Vorauswertung durch, falls noch nicht geschehen

'''

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("path_HO", help="The path to the main folder (Hauptordner)")
    # parser.add_argument("id", help="The id that is used (e.g. p for p_k)")

    # parser.add_argument("id_index", help="The index where to find the index when splitting the ho_name by '_'", type=int)

    # parser.add_argument("--read_tracking", help="Bool if tracking files should be read in", action='store_true')
    # parser.add_argument("--read_lp", help="Bool if lp_variables should be read in", action='store_true')

    args  = parser.parse_args()

    # print(args)

    rsb_vorauswertung(args.path_HO)
