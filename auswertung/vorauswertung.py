from auswertung_ma import Read_HO
import argparse
import os

'''
Skript, das mich Datenauswertung auf dem Cluster durchfuehren laesst

'''

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("path_HO", help="The path to the main folder (Hauptordner)")
    parser.add_argument("id", help="The id that is used (e.g. p for p_k)")

    parser.add_argument("id_index", help="The index where to find the index when splitting the ho_name by '_'", type=int)

    parser.add_argument("--read_tracking", help="Bool if tracking files should be read in", action='store_true')
    parser.add_argument("--read_lp", help="Bool if lp_variables should be read in", action='store_true')

    args  = parser.parse_args()

    # print(args)

    data = Read_HO(args.path_HO, args.id, args.id_index, read_tracking=args.read_tracking, read_lp=args.read_lp)
    save_lp = True if args.read_lp else False

    # falls ich auf meinm lokalen rechner bin, dann nehme lokalen Vorauswertungspfad
    save_path = r'D:\Uni\Masterarbeit\Daten\Vorauswertung' if os.path.exists(r'D:\Uni') else r'/gss/work/xees8992/Vorauswertung'

    data.save_results(save_path, save_lp=save_lp)