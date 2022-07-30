import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from shutil import move, rmtree
import os
from shapely.geometry import Point, LineString
from shapely.geometry.collection import GeometryCollection

from functools import reduce

def read_tracking_files_sim(p, appendix, read_check=False, read_opt=False, read_lp=False, read_constr=False):
    """Laedt Tracking Datein fuer einen simulation_x Ordner

    Args:
        p (_type_): Pfad zu einem Simulationsordner
        read_ckeck (bool): gibt an, ob den check_ordner (Optimum) mit eingelesen werden sollen (default=False, weil die Datein darin sehr gross werden koennen)
        read_opt (bool): gibt an, ob den opt_ordner (LP_Approx) mit eingelesen werden sollen (default=False, weil die Datein darin sehr gross werden koennen)
        read_lp (bool): gibt an, ob ich die LP-Variablen einlesen moechte, zB. um zu schauen, wie viel Prozenta davon ganzzahlig sind
        read_constr (bool): gibt an, ob ich die Zeiten einlesen moechte, die in den Iterationen zum Erzeugen der Min-Cut-Constraints gebraucht werden
        appendix ... Das, was an read_lp_results (via calc_anteil_ganzzahliger_variablen) weitergegen werden soll, und da an die tmp.txt angehaendt werden soll, 
                     damit mehrere Skript gleichzeitig laufen koennen und sich nicht gegenseitig die tmp.txt Datein wegloeschen

    - Wenn read_opt True ist, dann lese ich die Opt-Zeiten ein, addiere sie zusammen und rechne sie von Millisekunden um in Sekunden
    - Die Gesamtlaufzeit vom LP-Alg (Spalte totale_lp_ms) rechne ich am Ende noch um in Sekunden

    Problem: in neueren Versionen, speicher ich setup-Zeiten unter 'setup_zeiten_ms' ab, daher kennt die Fkt. jetzt beides und liest das ein, was vorhanden ist

    """

    pt = os.path.join(p, "Tracking")

    # checke, ob es den Tracking-Ordner gibt
    if not os.path.exists(pt):
        raise ValueError("ROBERTERROR: Der Tracking-Ordner existiert nicht!")

    # checken, welche setup_zeit-Datei vorliegt
    # setup_filename = 'setup_zeiten_ms.txt' if os.path.exists(os.path.join(pt, 'setup_zeiten_ms.txt')) else 'setup_zeiten_s.txt'

    df = pd.DataFrame()

    dic_columnname_to_filename = {'optimum_loop_counter.txt': 'loop_counter',
                                'optimum.txt': 'total_optimum_alt[s]',
                                'optimum2.txt': 'total_optimum[s]',
                                'total_lp_ms.txt': 'total_lp_ms',
                                'approx_zeiten_s.txt': 'approx_teil[s]',
                                'counters.txt': 'lp_constraint_counter',
                                'loop_zeiten_s.txt': 'lp_teil_loop[s]',
                                'setup_zeiten_s.txt': 'lp_teil_setup[s]',
                                'setup_zeiten_ms.txt': 'lp_teil_setup[ms]'}
    dic_dtype_to_filename = {'optimum_loop_counter.txt': int,
                            'counters.txt': int}

    # lese erstmal alle Tracking Datein ein, die sich nicht in einem Unterordner befinden
    tracking_files = [f for f in os.listdir(pt) if os.path.isfile(os.path.join(pt, f))]

    for f in tracking_files:

        # falls ein non-default (default ist float) dtype hinterlegt ist:
        if dic_dtype_to_filename.get(f):
            df[dic_columnname_to_filename[f]] = np.loadtxt(os.path.join(pt, f), dtype=dic_dtype_to_filename[f])
        else:
            df[dic_columnname_to_filename[f]] = np.loadtxt(os.path.join(pt, f))

    if read_check:
        pt_check = os.path.join(pt, "check")

        # checken, ob es den check-Ordner gibt
        if not os.path.exists(pt_check):
            raise ValueError('ROBERTERROR: Der check-Ordner existiert nicht!')

        # falls er existiert, werden die Datein in der richtigen Reihenfolge (1.txt gehoert zu 2. Simulation) eingelesen und direkt weiterverarbeitet
        check = [np.loadtxt(os.path.join(pt_check, i)) for i in sorted(os.listdir(pt_check), key=lambda x: int(x.split(".")[0]))]
    
        sums = [sum(i) for i in check]
    
        df['sum_check[s]'] = sums
    
    if read_opt:
        pt_opt = os.path.join(pt, 'opt')

        # checken, ob es den check-Ordner gibt
        if not os.path.exists(pt_opt):
            raise ValueError('ROBERTERROR: Der opt-Ordner existiert nicht!')

        opt = [np.loadtxt(os.path.join(pt_opt, i)) for i in sorted(os.listdir(pt_opt), key=lambda x: int(x.split("_")[0]))]
        # umrechnen in Sekunden
        sums = [np.sum(i)/1000 for i in opt]        # nutze hier np.sum, weil sonst gibt es einen error, falls die opt-arrays nur einen Eintrag enthalten
    
        df['sum_opt[s]'] = sums

    if read_constr:
        pt_constr = os.path.join(pt, 'add_constr')

        # checken, ob es den Ordner gibt
        if not os.path.exists(pt_constr):
            raise ValueError('ROBERTERROR: Der add_constr-Ordner existiert nicht!')

        add_constrs = [np.loadtxt(os.path.join(pt_constr, i)) for i in sorted(os.listdir(pt_constr), key=lambda x: int(x.split("_")[0]))]
        sums = [np.sum(i) for i in add_constrs]
        df['sum_add_constr[s]'] = sums

    if read_lp:
        pt_lp = os.path.join(pt, 'lp_results')

        # checken, ob es den lp_results-Ordner gibt
        if not os.path.exists(pt_lp):
            raise ValueError('ROBERTERROR: Der lp_results-Ordner existiert nicht!')

        lp_res_prozent = [calc_anteil_ganzzahliger_variablen(os.path.join(pt_lp, i), appendix) for i in sorted(os.listdir(pt_lp), key=lambda x: int(x.split(".")[0]))]

        df['anteil_lp_int'] = lp_res_prozent

    # falls die total_lp_ms Spalte drin ist, rechne ich die noch in Sekunden um
    if 'total_lp_ms' in df.columns:
        df['total_lp[s]'] = df['total_lp_ms'] / 1000
        df.drop('total_lp_ms', axis='columns', inplace=True)

    return df

def read_tracking_files(konfig_path, appendix, read_check=False, read_opt=False, read_lp=False, read_constr=False):
    """Ist dafuer da, fuer eine Konfiguration die Trackingdaten aller Simulationen einzulesen und in ein Dataframe zu packen

    Args:
        konfig_path (_type_): Pfad zu dem Konfigurationsordner
        appendix ... Das, was an read_lp_results weitergegen werden soll, und da an die tmp.txt angehaendt werden soll, 
                     damit mehrere Skript gleichzeitig laufen koennen und sich nicht gegenseitig die tmp.txt Datein wegloeschen
    """    
    
    # muss erstmal rausfinden, welche Simulationen alle gemacht wurden
    sims = os.listdir(konfig_path)

    # erstes Dataframe, an das dann angebaut wird
    df = read_tracking_files_sim(os.path.join(konfig_path, sims[0]), appendix, read_check=read_check, read_opt=read_opt, read_lp=read_lp, read_constr=read_constr)
    
    for sim in sims[1:]:

        # nehme die results und packe sie an df dran
        df_sim = read_tracking_files_sim(os.path.join(konfig_path, sim), appendix, read_check=read_check, read_opt=read_opt, read_lp=read_lp, read_constr=read_constr)
        # dabei gehe ich aus, dass selbst, wenn die results in unterschiedlichen Spalten gespeichert werden, dass
        # die an die dazugehoerige, bereits vorhandene Spalte angegliedert werden
        df = pd.concat([df, df_sim], ignore_index=True)
    
    return df

# muss ich noch umschreiben, benutze ich zur zeit auch noch nicht
def read_hauptordner(path):
    """_summary_

    Args:
        path (_type_): path ist der Pfad zu dem Haupt-Ordner, wo die Konfigurationsordner drin liegen

    """
        
    # jetzt alle Ergebnisse einlesen
    konfigs = os.listdir(path)
    
    # dictionary, was mir fuer die verschiedenen Simulationen den Index zum relevanten Parameter gibt
    # Fuer TreePlusP, ist hier die Wahrscheinlichkeit p relevant
    d = {"TreePlusP": 1}
    
    df_res = pd.DataFrame()
    
    for k in konfigs:
        p = float(k.split("_")[d[k.split("_")[0]]])
        df = read_files(os.path.join(path, k))
        
        di = {'p': p}
        di.update({alg: df.mean()[alg] for alg in df.columns})
#         return pd.DataFrame(di, index=[0])
        df_res = pd.concat([df_res, pd.DataFrame(di, index=[0])], ignore_index=True)
    
    return df_res

def read_problems(path_prob, calc_ev=True):
    """liest Problem files ein (dazu gehoeren die Scenariowahrscheinlichkeiten und die erzeugten Kantengewichte)

    Args:
        path_prob (_type_): path zu einem Problems-Ordner
        calc_ev (bool): gibt an, ob ich fuer die Kanten den EV fuer die 2. stage ausrechnen und an das scenarios df anhaengen soll
    """

    # lese in der richtigen Reihenfolge ein
    number_sims = int(len(os.listdir(path_prob))/2)
   
    scenarios = [pd.read_csv(os.path.join(path_prob, f"scenarios{i}.csv"), skipinitialspace=True).iloc[:, :-1].set_index('EdgeID') for i in range(number_sims)]
    probs = [pd.read_csv(os.path.join(path_prob, f"scenario_probs{i}.csv"), names=['Wahrscheinlichkeit']) for i in range(number_sims)]

    if calc_ev:
        for i in range(number_sims):
            scenarios[i]['EV'] = (scenarios[i].loc[:, 'scenario0': scenarios[i].columns[-1]].values * probs[i]['Wahrscheinlichkeit'].values).sum(1)


    return probs, scenarios

# Fkt zum Einlesen der Edges
def read_mean_edge_number(sim_path):
    """Liest mir aus einem simulation_x-Ordner die edges_count-Datei ein und gibt den Mittelwert davon zurueck

    Args:
        sim_path (string): Pfad zu einem simulation_x Ordner, aus dem die edges_count-Datei ausgewertet werden soll

    Returns:
        _type_: mittlere Anzahl an Edges
    """

    a = np.loadtxt(os.path.join(sim_path, 'edges_count.txt'))
    return a.mean()

# die Optimum maps einlesen
def readLGF_Network(source):
    mInFile = open(source, mode='r')
    fileString = mInFile.read()
    mInFile.close()
    if '@arcs' in fileString:
        initKey = '@arcs'
        initPos = 6
    elif '@edges' in fileString:
        initKey = '@edges'
        initPos = 7
    else:
        print('No keyword \'@arcs\' or \'@edges\' found\n Wrong file format')
        return
    strDat = fileString[fileString.find(initKey)+initPos:].split("\n", 1)[1]
    
    df = pd.DataFrame(columns=['u', 'v', 'id', 'first_stage'])
    
    for row in strDat.split('\n')[:-1]:
        s = row.split("\t")
        df = df.append({'u': int(s[0]), 'v': int(s[1]), 'id': int(s[2]), 'first_stage': int(s[3])}, ignore_index=True)
    return df

def read_lp_results(source, appendix):
    """liesst mir von EINEM BELIEBIGEN ALG die Ergebnismaps ein und returned den relevanten Teil davon als Array

    Args:
        source (_type_): path zu der lp_results Datei
        appendix .. Das, was an das tmp.txt angehaendt werden soll, damit mehrere Skript gleichzeitig laufen koennen und sich nicht gegenseitig die tmp.txt Datein wegloeschen

    Returns:
        _type_: Anteil aller LP-Variablen, die ganzzahlig sind
    """

    mInFile = open(source, mode='r')
    fileString = mInFile.read()
    mInFile.close()
    if '@arcs' in fileString:
        initKey = '@arcs'
        initPos = 6
    elif '@edges' in fileString:
        initKey = '@edges'
        initPos = 7
    else:
        print('No keyword \'@arcs\' or \'@edges\' found\n Wrong file format')
        return

    # versuche, die relevanten Daten als np_array aus zwischengespeichertem tmp.txt-File einzulesen
    try:
        with open(f'tmp_{appendix}.txt', 'w') as tmp:
            tmp.write(fileString[fileString.find(initKey)+initPos:].split("\n", 1)[1])
        
        arr = np.loadtxt(f'tmp_{appendix}.txt')
        os.remove(f'tmp_{appendix}.txt')
    except:
        # falls es einen Fehler gegeben hat
        print('ROBERTERROR: Es gab einen Fehler beim Erzeugen des np-Arrays!')
        
        try:
            os.remove(f'tmp_{appendix}.txt')
        except FileNotFoundError:
            print('ROBERTERRor: Die tmp.txt Datei war bereits gelöscht!')
    
    return arr

def calc_anteil_ganzzahliger_variablen(lp_res_source, appendix):
    """_summary_

    Args:
        lp_res_source (_type_): _description_
        appendix ... Das, was an read_lp_results weitergegen werden soll, und da an die tmp.txt angehaendt werden soll, 
                     damit mehrere Skript gleichzeitig laufen koennen und sich nicht gegenseitig die tmp.txt Datein wegloeschen
    Returns:
        _type_: _description_
    """

    arr = read_lp_results(lp_res_source, appendix)

    # nimm von dem array nur den Teil in dem die LP-Variablen drin stehen und checke, wie viele davon integer sind
    at = arr[:, 3:]

    # wen modulo 1 groesser als 0 ist, dann ist es nicht ganzzahlig, ich rechne also erst den Anteil der nicht ganzzahligen lsg aus und ziehe das dann von 1 ab
    return 1 - ((at[np.mod(at, 1)>0]).size / at.size)

# rechnet mir zu einem geg. p (Wahrscheinlichkeit pro Kante) aus, wie viele Kanten in mit meinem TreePlusP Konstruktor zu erwarten habe
def calc_expected_edges_p(n, p):
    if n < 2:
        raise ValueError("n muss größer als 1 sein!")
    if p < 0 or p > 1:
        raise ValueError("p muss Wahrscheinlichkeit sein!")
    
    return n-1 + p * ((n**2-3*n+2)/2)

# rechnet mir das p aus, das ich in calc_expected_edges einsetzen kann, und zwar zu einem geg. c so, dass ich am Ende einen Erwartungswert fuer die Anzahl an Kanten von c * (n-1)/2 habe 
def calc_p_to_c(n, c):
    if c < 2:
        print('Warnung, da c<2 ist, wird p=0 gesetzt!')
        return 0
    if c > n:
        raise ValueError("c darf nicht größer als n sein!")
    
    return (c * (n-1) - 2*n + 2) / (n**2-3*n+2)

# rechnet mir die zu erwartende Kantenanzahl aus, die das c im erdos renyi graphen erzeugen soll
def calc_expected_m(n, c):
    return c * (n-1) / 2

# exponentieller Fit:
def f_exp(x, a, b):
    return a * np.exp(b *x)

# polynomialer Fit:
def f_poly(x, a, b):
    return a * (x ** b)

# polynomialer Fit mit mehr als nur einem Term
def f_poly_long(x, a, b, c):
    return a * x**b + c 

def calc_anteil_ganz_geloest(df):
    """gibt mir von einem eingelesenen df aus der read_tracking_files- Funktion an, wie viel Prozent aller Loesungen des LP-Teils bereits ganzzahlig sind

    Args:
        df (dataframe): Ergebnis df der 'read_tracking_files'-Funktion
    """

    if (df[df['anteil_lp_int'] == 0].shape[0] > 0):
        raise ValueError(f"ROBERTERROR: Es gibt {df[df['anteil_lp_int'] == 0].shape[0]} Instanz(en), wo alle LP-Variablen nicht ganzzahlig sind!")

    return df[df['anteil_lp_int'] == 1].shape[0] / df.shape[0]

def read_results(konfig_path):
    """liest mir die results.txt Dateien zu allen Simulationen einer Konfig ein und gibt diese als Dataframe zurueck

    Args:
        konfig_path (_type_): _description_
    """
    # muss erstmal rausfinden, welche Simulationen alle gemacht wurden
    sims = os.listdir(konfig_path)

    # versuche so lange ein df anzufangen, bis es in einem simulation Ordner eine results datei gibt
    i = 0
    while(True):

        # falls ich alle sim-Ordner durchprobiert habe
        if i >= len(sims):
            break

        try:
            df = pd.read_csv(os.path.join(konfig_path, sims[i], "results.txt")).iloc[:,:-1]
            break
        except FileNotFoundError:
            name = os.path.split(konfig_path)[1] + "\\" +  sims[0]
            print(f'ROBERTWARNUNG: Im Ordner {name} gibt es keine results.txt Datei!')
    
        i += 1

    # erstes Dataframe, an das dann angebaut wird
    # try: 
    #     df = pd.read_csv(os.path.join(konfig_path, sims[0], "results.txt")).iloc[:,:-1]

    # except FileNotFoundError:
    #     name = os.path.split(konfig_path)[1] + "\\" +  sims[0]
        # print(f'ROBERTWARNUNG: Im Ordner {name} gibt es keine results.txt Datei!')
    for sim in sims[i+1:]:
        
        # falls im ersten Ordner schon keine results datei liegt, dann wuerde hier der naechste error kommen, aber das lasse ich erstmal so
        try:

            # nehme die results und packe sie an df dran
            df_sim = pd.read_csv(os.path.join(konfig_path, sim, "results.txt")).iloc[:,:-1]
            # dabei gehe ich aus, dass selbst, wenn die results in unterschiedlichen Spalten gespeichert werden, dass
            # die an die dazugehoerige, bereits vorhandene Spalte angegliedert werden
            df = pd.concat([df, df_sim], ignore_index=True)

        except FileNotFoundError:
            name = os.path.split(konfig_path)[1] + "\\" +  sim
            print(f'ROBERTWARNUNG: Im Ordner {name} gibt es keine results.txt Datei!')

    return df

def read_vorauswertung(path_vor, id, id_stelle, read_lp=False):
    """
    liest einen Vorauswertungsordner komplett ein und gibt mir ein benutzerfreundliches Dictionary zurueck

    """

    dic = {}

    for ho in os.listdir(os.path.join(path_vor, 'Alg_Results')):
        n = int(ho.split("_")[1])

        # zur Sicherheit, damit nicht irgendwelche wilden Kombinationen entstehen
        if n in dic.keys():
            raise ValueError (f"ROBERTERROR: N={n} kommt in mehreren Hauptordnern vor, das sollte lieber eindeutig sein!")

        dic[n] = Read_HO(os.path.join(path_vor, 'Alg_Results', ho), id, id_stelle, read_vorauswertung=True)

    # lese die lp_Ergebnisse ein
    if read_lp:

        dic_lp = {}

        for ho in os.listdir(os.path.join(path_vor, 'LP_Results')):
            n = int(ho.split("_")[1])

            # addiere die jeweiligen Werte der gleichen Ids zusammen und berechne daraus die entsprechenden Anteile
            df = pd.read_csv(os.path.join(path_vor, 'LP_Results', ho, 'simulation_0.csv'))
                
                    # addiere fuer alle anderen simulationen die entsprechenden werte zu bestehenden hinzu oder fuege neue Zeilen dem df hinzu
            sims = [pd.read_csv(os.path.join(path_vor, 'LP_Results', ho, s)) for s in sorted(os.listdir(os.path.join(path_vor, 'LP_Results', ho)))[1:]]
            df = pd.concat([df, *sims]).groupby('ids').sum()

            df['Anteil_GGL'] = df['Anzahl_GGL'] / df['Runs']
            df['Anteil_GLPV'] = df['Summe_LPV'] / df['Runs']
            dic_lp[n] = df

        return dic, dic_lp

    return dic

# dazu da, alle Daten auf einmal einzulesen
def read_ueberordner():
    pass


class Read_HO:
    """

    ho ist der Ordner, wo die ganzen Konfigordner drin liegen

    id: p steht fuer p_k

    """

    def __init__(self, path_ho, id, id_stelle, read_vorauswertung=False, read_lp=True, read_tracking=True):
        """
        habe zwei Moeglichkeiten, die Daten einzulesen, beide muessen garantieren, dass alle Folgemethoden korrekt funktionieren

        path_ho ist der Pfad zum Hauptordner im Alg_Results-Ordner
        """

        self.path_ho = path_ho

        # lese bereits berechnete Vorauswertung ein, dabei gibt es bisher keine Tracking Daten!
        if read_vorauswertung:

            # wenn die Daten bereits vorausgewertet sind, dann steht die id am Ende (vor der ID-Art) der raw-results csv Dateien
            self.id_stelle = -2
            self.id = id
            self.id_type = int if id == 'n' else float

            self.raw_results = {}

            # erstmal die Daten aus dem ersten sim-Ordner einlesen
            for raw_result in os.listdir(os.path.join(self.path_ho, 'simulation_0')):

                self.raw_results.update({float(raw_result.split('_')[self.id_stelle]): pd.read_csv(os.path.join(self.path_ho, 'simulation_0', raw_result))})


            for sim in sorted(os.listdir(self.path_ho))[1:]:
                for raw_result in os.listdir(os.path.join(self.path_ho, sim)):
                    
                    id_next = float(raw_result.split('_')[self.id_stelle])

                    if id_next in self.raw_results:

                        # an bereits bestehendes dataframe angliedern
                        df = pd.read_csv(os.path.join(self.path_ho, sim, raw_result))
                        self.raw_results[id_next] = pd.concat([self.raw_results[id_next], df], ignore_index=True)
                        pass
                    else:
                        self.raw_results.update({id_next: pd.read_csv(os.path.join(self.path_ho, sim, raw_result))})

            self.id_values = sorted(self.raw_results.keys())

        else:

            if id not in ('c', 'n', 'p', 'b_beide'):
                raise ValueError ("ROBERTERROR, die id muss entweder c oder n sein!")

            self.id_stelle = id_stelle
            self.id = id
            self.id_type = int if id == 'n' else float
            self.id_tups = [(self.id_type(f.split('_')[id_stelle]), f) for f in os.listdir(path_ho)]
            self.id_values = [id for id, _ in self.id_tups]

            self.appendix = os.path.split(self.path_ho)[1].split('_')[1]
            # print('Appendix: ', self.appendix)


            print('Lese die TrackingDaten ein...')
            self.dfs = {id: read_tracking_files(os.path.join(self.path_ho, f), self.appendix, read_lp=read_lp) for id, f in self.id_tups}      # hier stehen die tracking daten drin
            print('fertig!')


            if read_lp:
                self.anteil_ganz_geloest = [calc_anteil_ganz_geloest(self.dfs[id]) for id in self.id_values]
                self.mean_anteil_lp_ganz = [self.dfs[id].mean()['anteil_lp_int'] for id in self.id_values]

                if read_tracking:
                    # weitere Trackingwerte
                    self.mean_total_lp = [self.dfs[id].mean()['total_lp[s]'] for id in self.id_values]
                    self.mean_loop_iterations = [self.dfs[id].mean().lp_constraint_counter for id in self.id_values]

            # will auch die Algorithmenergebnisse vergleichen
            self.m_res = read_alg_performances(self.path_ho, id_stelle)

            self.raw_results = {}

            for k in os.listdir(self.path_ho):

                try:
                    self.raw_results.update({float(k.split('_')[self.id_stelle]): read_results(os.path.join(self.path_ho, k))})

                except UnboundLocalError:
                    print(f'ROBERTWARNUNG: Im Ordner {k} gibt es keine results.txt Dateien!')

    # def __add__(self, other):
    #     """
    #     baue aus zwei Read_HOs ein neues, das soll zum Zusammenfuehren von lokalen und vorausgewerteten Daten benutzt werden
        
    #     """
    #     pass

    def calc_mean_alg_results(self):
        """
        rechnet mir die mittleren Ergebnisse der einzelnen Algorithmen aus
        """

        # gehe alle raw_results durch

        df = pd.concat([self.raw_results[i].mean().to_frame().T for i in self.id_values])
        df['ids'] = self.id_values

        return df.set_index('ids')

        # return self.raw_results

    def calc_statistic_size(self):
        """
        rechnet mir aus, wie viele Simulationen es zu den jeweiligen ids gibt
        """
        
        df = pd.DataFrame()

        df['ids'] = self.id_values
        df['stat_size'] = [self.raw_results[id].shape[0] for id in self.id_values]

        return df.set_index('ids')

    def calc_variance(self, prop):
        """
        berechnet die Varianz der Proportion prop (zB. Anteil der Faelle bei denen LP_Approx die Schranke4b * alpha erreicht)

        prop muss in der Reihenfolge der dazugehoerenden IDs sortiert sein
        """

        df_stat = self.calc_statistic_size()
        p = np.array(prop)
        return p * (1 - p)

    def calc_std_deviation(self, prop, id_subset=None):
        """
        berechnet die Standardabweichung der Proportion prop (zB. Anteil der Faelle bei denen LP_Approx die Schranke4b * alpha erreicht)

        prop muss sortiert sein, in der Reihenfolge der dazugehoerenden IDs

        falls ein id_subset uebergeben wird, dann sollen nur die stats zu den uebergebenen ids zureuck gegeben werden
        """

        df_stat = self.calc_statistic_size()

        if id_subset:
            # gleiche die Ids im Subset mit den eigenen ab
            ids = sorted(set(self.id_values).intersection(set(id_subset)))
            df_stat = df_stat.loc[ids]

        p = np.array(prop)

        return np.sqrt(np.array(p*(1-p) / df_stat['stat_size']))


    def plot_results(self, **kwargs):
        fig, ax = plt.subplots(figsize=(16,10))

        for alg in self.m_res:
            ax.scatter(self.m_res[alg]['ids'], self.m_res[alg]['ms'], label=alg, **kwargs)
        ax.legend()
        ax.set_title("Knoten mit hpc$", fontsize=15)
        # ax.set_xlabel("Wahrscheinlichkeit p eine Kante zu einem Baum hinzuzufügen")
        ax.set_ylabel("im Mittel erzielter Erwartungswert")
        return ax

    # liest mir die results ein und schaut, in wieviel Prozent der Faelle, ein Approx-Alg kleiner war als alpha mal Schranke4b
    def check_alg_vs_schranke4b(self, alg, alpha):

        if alg not in ('LP_Approx', 'Greedy'):
            raise ValueError ('ROBERTERROR: Unterstuetze nur LP_Approx und Greedy!')

        if alpha < 1.0:
            print('ROBERTWARNUNG: Alpha sollte groesser als 1 sein!')
        
        ids = []
        proz = []

        diffs = []

        for id in sorted(self.raw_results.keys()):

            try:
                # rechne aus, in wieviel Prozent der Faelle der Approx_Alg kleiner gleich alpha mal Schranke 4b war
                df = self.raw_results[id].dropna(subset=[alg])      # wichtig: ich darf hier nur mit dem Teil der Ergebnisse arbeiten, wo der entsprechende alg mit drin vorkommt
                # print(df)
                proz.append(df[df[alg] <= alpha * df['Schranke4b']].shape[0] / df.shape[0])

                ids.append(id)
            
                # errechnet die mittlere differenz aus alg-Ergebnis und Schranke4b-Ergebnis
                diffs.append((df[alg] / df['Schranke4b']).mean())

            except KeyError:
                # print(f'ROBERTERROR: Keyerror mit Alg {alg} fuer die ID {id}, wahrscheinlich gibt es in dieser Simulation keinen Eintrag zum Alg {alg}!')
                pass

        return ids, proz, diffs

    def save_results(self, save_path, save_lp=False):
        """
        speichere mir die Ergebnisse der Auswertung, dazu Dataframes mit den raw results und zwar wieder im simulation_x Format, damit ich mehrere verschiedene Simulationen 
        zusammen packen und auswerten kann

        speichere ausserdem wenn gewuenscht die lp-Ergebnisse ab, ebenfalls im simulation_x Format

        """
        name_dir = os.path.split(self.path_ho)[1]
        res_path = os.path.join(save_path, 'Alg_Results')

        # falls so eine Auswertung bereits existert, zaehle die simulations ordner und packe den naechsten dazu
        if os.path.exists(os.path.join(res_path, name_dir)):
            c = len([sim for sim in os.listdir(os.path.join(res_path, name_dir)) if 'simulation_' in sim])
            sim_path = os.path.join(res_path, name_dir, f'simulation_{c}') 
            

        else:
            os.mkdir(os.path.join(res_path, name_dir))

            # erstelle den Simulationsordner 0
            sim_path = os.path.join(res_path, name_dir, 'simulation_0') 

        os.mkdir(sim_path)

        for id in self.raw_results:
            self.raw_results[id].to_csv(os.path.join(sim_path, f'raw_results_{id}_{self.id}.csv'), index=False)

        # falls lp-Ergebnisse mit gespeichert werden sollen
        if save_lp:
            lp_path = os.path.join(save_path, 'LP_Results')

            # falls so eine Auswertung bereits existert, zaehle die simulations ordner und packe den naechsten dazu
            if os.path.exists(os.path.join(lp_path, name_dir)):
                c = len([sim for sim in os.listdir(os.path.join(lp_path, name_dir)) if 'simulation_' in sim])
                # sim_path = os.path.join(lp_path, name_dir, f'simulation_{c}') 
                sim = c
            
            else:
                os.mkdir(os.path.join(lp_path, name_dir))

                # erstelle den Simulationsordner 0
                # sim_path = os.path.join(lp_path, name_dir, 'simulation_0') 
                sim = 0

            # os.mkdir(sim_path)

            df = pd.DataFrame()
            df['ids'] = self.id_values

            # Anzahl ganz geloester Problemstellungen
            df['Anzahl_GGL'] = [self.dfs[id][self.dfs[id]['anteil_lp_int'] == 1].shape[0] for id in self.id_values]

            # Summe der Anteile ganzzahliger LP-Variablen
            df['Summe_LPV'] = [self.dfs[id].sum()['anteil_lp_int'] for id in self.id_values]

            # die Samplegroesse
            df['Runs'] = [self.dfs[id].shape[0] for id in self.id_values]

            df.to_csv(os.path.join(lp_path, name_dir, f'simulation_{sim}.csv'), index=False)

            # self.anteil_ganz_geloest = [calc_anteil_ganz_geloest(self.dfs[id]) for id in self.id_values]
            # self.mean_anteil_lp_ganz = [self.dfs[id].mean()['anteil_lp_int'] for id in self.id_values]


# liesst mir die perfomances der verschiedenen Algorithmen ein, die im Hauptordner ho liegen und wo die Konfigurationen nach der id (zb. c) durchgegangen werden
def read_alg_performances(ho, id_index):
    
    # lese die Ergebnisse aller simulationen ein
    # dfs = [read_results(os.path.join(ho, k)) for k in os.listdir(ho)]

    # ids = [float(k.split('_')[id_index]) for k in os.listdir(ho)]
    dfs = []
    ids = []

    for k in os.listdir(ho):

        try:
            dfs.append(read_results(os.path.join(ho, k)))
            ids.append(float(k.split('_')[id_index]) )
        except UnboundLocalError:
            print(f'ROBERTWARNUNG: Im Ordner {k} gibt es keine results.txt Dateien!')

#     return dfs[0]
    m_dic = {}
    for i, df in enumerate(dfs):
        for alg in df.columns:
            if alg in m_dic:
                # kann es zu Algorithmen verschieden viele Simulationen bei verschiedenen c geben?
                # ja, (zB wenn der opt-Alg nur fuer die ersten paar c laeuft)
                m_dic[alg]['ids'].append(ids[i])
                m_dic[alg]['ms'].append(df.mean()[alg])
#                 m_dic[alg]['stds'].append(stds[i][alg])
            else:
                # print(i, alg)
                m_dic.update({alg: {'ids':[ids[i]], 'ms':[df.mean()[alg]]}})
    
    return m_dic

# nicht fertig geschrieben, da ich fuer die Berechnung noch die Problemstellungen immer extra abspeichern muesste, daher ist das jetzt in cpp geloest
def calc_lp_schranke(lp_res_source):
    
    arr = read_lp_results(lp_res_source)

    # nimm von dem array nur den Teil in dem die LP-Variablen drin stehen und checke, wie viele davon integer sind
    at = arr[:, 3:]

    s = 0

    # gehe ueber alle Kanten und alle Szenarien
    for e in range(at.shape[0]):
        pass
    
def einordnen(src, dst):
    """
    Fkt. um neue Simulationsergebnisse von einem Ordner in bisher bestehenden Ordner einzuordnen
    
    src ist dabei der Pfad zu dem Ordner, wo die neuen Simulationsergebnisse liegen
    
    dst ist Pfad zu dem Ordner mit dem selben Namen in dem schon die ganzen vorherigen Simulationen drin liegen und in dessen Unterordner die neuen Simulationen einsortiert werden 
    sollen
    
    """
    
    for ordner_src in os.listdir(src):
        if ordner_src in os.listdir(dst):
            print(f'{ordner_src} gibt es bereits, verschiebe Simulationsordner.')
            number_sims_dst = len(os.listdir(os.path.join(dst, ordner_src)))
            move(os.path.join(src, ordner_src, 'simulation_0'), os.path.join(dst, ordner_src, f'simulation_{number_sims_dst}'))
        else:
            print(f'Den Ordner {ordner_src} gab es noch nicht, ich verschiebe ihn nach dst!')
            # verschiebe den Ordner einfach
            move(os.path.join(src, ordner_src), os.path.join(dst, ordner_src))
    
    # entferne den src Ordner, nachdem alle Datein uebertragen wurden
    rmtree(src)

def calc_schnittpunkte(ids1, y1, ids2, y2):
    """
    Rechnet mit Hilfe von shapely LineString-Objekten die Schnittpunkte der beiden uebergebenen Kurven aus
    Wichtig: es werden die Schnittpunkte der Linen ermittelt, die sich ergeben, wenn die einzelnen Punkte der Kurven verbunden werden

    """
    
    # unnoetig?
    # ids1 = np.array(ids1)
    # ids2 = np.array(ids2)
    # y1 = np.array(y1)``
    # y2 = np.array(y2)

    
    first_line = LineString(np.column_stack((ids1, y1)))
    second_line = LineString(np.column_stack((ids2, y2)))
    intersection = first_line.intersection(second_line)
    # print(intersection)

    # es gibt keinen Schnittpunkt
    if intersection.is_empty:
        return None

    # es gibt genau einen Schnittpunkt
    elif isinstance(intersection, Point):
        return intersection.x, intersection.y

    # es gibt mehrere Schnittpunkte
    else:
        # nehme die LineStrings raus, falls es eine GEOMETRYCOLLECTION ist, ansonsten packe ich alle geometry-Objekte (zB Points eines MultiPoints) so in 
        # eine Liste zur leichteren Verarbeitung
        l = [g for g in intersection.geoms if not isinstance(g, LineString)]

        xs = [p.x for p in l]
        ys = [p.y for p in l]

        # falls nur ein Point uebrig bleibt
        if len(xs) == 1 and len(ys) == 1:
            return xs[0], ys[0]
        else:
            return xs, ys
    
    # es gibt genau einen Schnittpunkt
    # elif isinstance(intersection, Point):
        # return intersection.x, intersection.y
    
    # es gibt mehrere Schnittpunkte
    # else:
    #     xs = [p.x for p in intersection.geoms]
    #     ys = [p.y for p in intersection.geoms]
    
    # return xs, ys

class PlotCollector:
    """
    speichert mir die relevanten Daten zum plotten

    std_dev ist eher der Standardfehler
    """

    def __init__(self, _ids, _props, _std_dev, _var):

        self.ids = _ids
        self.props = _props
        self.std_dev = _std_dev
        self.var = _var

def read_all_data(p_lokal, p_vor, id_name='p', id_stelle=-2):
    """
    liest mir die alten (lokalen) Daten und die auf dem Cluster vorausgewerteten Daten zusammen ein und gibt sie mir fertig zum plotten in Form eines DataFrames zurueck

    p_lokal path zum Ueberordner (dort wo die ganzen Hauptordner (HO) drin liegen)
    p_vor   path zum Ueberordner der vorausgewerteten Daten

    """

    # Warnung, falls die beiden Ueberordner nicht gleich heissen
    if os.path.split(p_lokal)[1] != os.path.split(p_vor)[1]:
        print("ROBERTWARNUNG: Die beiden Ordnernamen stimmen nicht überein! Sicher, dass das so richtig ist?")

    # lese die lokalen Daten ein
    data = {}

    for ho in os.listdir(p_lokal):
        if not 'Vorauswertung' in ho:
            n = int(ho.split("_")[1])
            data[n] = Read_HO(os.path.join(p_lokal, ho), id_name, id_stelle, read_tracking=False, read_lp=False)
            
    # lese die vorausgewerteten Daten ein
    data_vor = {}

    for ho in os.listdir(p_vor):
        n = int(ho.split("_")[1])
        data_vor[n] = Read_HO(os.path.join(p_vor, ho), id_name, id_stelle, read_vorauswertung=True)

    return data, data_vor

def prepare_alg_vs_schranke_data(data, data_vor, alg, alpha):
    """
    baut mir fuer eingelesene Daten PlottCollectorobjekte zum leichten plotten
    """

    dic = {}

    # bringe lokale und vorausgewertete Daten zum selben n zusammen

    ns = sorted(data.keys())
    ns_vor = sorted(data_vor.keys())

    for n in set(ns + ns_vor):
        ids = []
        props = []
        std_dev = []
        var = []

        if n in ns:
            # fuege lokale Daten hinzu
            ids += data[n].check_alg_vs_schranke4b(alg, alpha)[0]
            props += data[n].check_alg_vs_schranke4b(alg, alpha)[1]
            std_dev += list(data[n].calc_std_deviation(data[n].check_alg_vs_schranke4b(alg, alpha)[1]))
            var += list(data[n].calc_variance(data[n].check_alg_vs_schranke4b(alg, alpha)[1]))

            if n in ns_vor:
                # n kommt auch in den vorausgewerteten Daten vor und ich haenge die Daten an

                ii, pp, dd =  data_vor[n].check_alg_vs_schranke4b(alg, alpha)

                ids += ii
                props += pp
                # ids += data_vor[n].check_alg_vs_schranke4b(alg, alpha)[0]
                # props += data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1]
                std_dev += list(data_vor[n].calc_std_deviation(data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1], id_subset=ii))
                var += list(data_vor[n].calc_variance(data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1]))

            # error, falls die selbe ID mehrfach vorkommt
            if len(set(ids)) != len(ids):
                raise ValueError(f'ROBERTERROR: Ein ID-Wert kommt bei N={n} mehrfach vor!')

            # ids sortieren und den rest dazu mit
        
            props = [t[1] for t in sorted(zip(ids, props))]
            std_dev = [t[1] for t in sorted(zip(ids, std_dev))]
            var = [t[1] for t in sorted(zip(ids, var))]
        
            ids = sorted(ids)

            # fuege die eingelesenen Daten zum dictionary hinzu    
            dic[n] = PlotCollector(ids, props, std_dev, var)

        # ansonsten stammt das n aus ns_vor und ich fuege es dem Dictionary hinzu (falls der alg in den neuen Daten ueberhaupt vorkommt)
        else:

            # Achtung: hier beruecksichtige ich nicht, dass es sein kann, dass innerhalb eines Hauptordners unterschiedliche Algorithmen bei den verschiedenen ids verwendet wurden
            if alg in data_vor[n].raw_results[data_vor[n].id_values[0]].columns:

                ids = data_vor[n].check_alg_vs_schranke4b(alg, alpha)[0]
                props = data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1]
                std_dev = list(data_vor[n].calc_std_deviation(data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1]))
                var = list(data_vor[n].calc_variance(data_vor[n].check_alg_vs_schranke4b(alg, alpha)[1]))

                # fuege die eingelesenen Daten zum dictionary hinzu    
                dic[n] = PlotCollector(ids, props, std_dev, var)

    return dic


def prepare_lp_data(data, data_vor):
    """
    baut mir die eingelesenen LP-Daten zusammen

    hab ich jetzt nicht implementiert, ich habe stattdessen die paar Daten die ich hatte vorausgewertet

    """
    pass

def plot_hist_alg_vs_schranke(l, l_ns, alg, id_value, xlim_max=4, alpha=.6):
    """
    erzeugt mit ein Histogram zu den Verhaeltnissen der Algwerte zu den Schranke4b Werten

    l .. Sequence of Read_HO-Objects
    l_ns .. Liste mit den Ns zum labeln

    alg .. the alg that should be investigated
    id_value .. Wert der ID

    """

    if alg not in ('LP_Approx', 'Greedy'):
        raise ValueError ('ROBERTERROR: Unterstuetze nur LP_Approx und Greedy!')

    fig, ax = plt.subplots(figsize=(16,10))

    vhs = [rho.raw_results[id_value][alg] / rho.raw_results[id_value]['Schranke4b'] for rho in l]

    max_vh = max([s.max() for s in vhs])

    for i, vh in enumerate(vhs):
        ax.hist(vh, bins=150, range=(1,max_vh), label=f'N={l_ns[i]}', alpha=alpha)
    
    ax.set_xlim([1, xlim_max])
    ax.legend()

def prepare_fssa(ls, pcs, ordnername):
    """
    bereitet mir die Daten zu den uebergebenen Ls (Ns) und den dazugehoerenden pcs (PlottCollectors) im Ordner mit dem uebergebenen Namen so vor, dass ich sie leicht
    in meinem alten fssa Skript einlesen kann

    """
    
    p = os.path.join(r'D:\Uni\Masterarbeit\Daten\fssa', ordnername)
    
    # kann nur id werte nehmen, die fuer alle L vorliegen
    common_ids = sorted(reduce(lambda a, b : a.intersection(b), [set(pcs[l].ids) for l in ls]))
    
    # a und da bauen
    al = []
    dal = []
    
    for l in ls:
        # gehe alle eigenen IDs durch und fuege zum selben Index die entsprechenden Werte (props, std_dev) hinzu, falls
        # die id auch in den common_ids vorkommt, sonst nicht
        al.append([pcs[l].props[i] for i, ii in enumerate(pcs[l].ids) if ii in common_ids])
        dal.append([pcs[l].std_dev[i] for i, ii in enumerate(pcs[l].ids) if ii in common_ids])
        
    # alles abspeichern
    np.savetxt(os.path.join(p, 'l.txt'),  np.array(sorted(ls)))
    np.savetxt(os.path.join(p, 'rho.txt'),  np.array(sorted(common_ids)))
    
    np.savetxt(os.path.join(p, 'a.txt'), np.array(al))
    np.savetxt(os.path.join(p, 'da.txt'), np.array(dal))

if __name__ == '__main__':

    alpha = 1.001
    alg = 'LP_Approx'

    data1 = {}
    p1 = r"D:\Uni\Masterarbeit\Daten\KFC2\1_scenario"

    for ho in os.listdir(p1):
    #     print(ho)
        if not 'Vorauswertung' in ho:
            n = int(ho.split("_")[1])
            data1[n] = Read_HO(os.path.join(p1, ho), 'p', -2, read_tracking=False, read_lp=False)

    pv = r'D:\Uni\Masterarbeit\Daten\KFC2\1_scenario\Vorauswertungen'
    data1_vor = read_vorauswertung(pv, id='p', id_stelle=-2)

    pcs1 = prepare_alg_vs_schranke_data(data1, data1_vor, alg, alpha)
