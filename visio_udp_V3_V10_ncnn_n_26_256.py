import os
# ==========================================
# 0. OPTIMITZACIÓ DE MAQUINARI (THREADS)
# ==========================================
# Limitem els fils d'execució de la CPU a 4 perquè la Raspberry Pi 
# no es col·lapsi intentant obrir massa processos paral·lels matemàtics.
os.environ["OMP_NUM_THREADS"] = "4"
os.environ["OPENBLAS_NUM_THREADS"] = "4"

import torch
from ultralytics import YOLO
import cv2
import math
import numpy as np
import itertools
import json
import sys
import time
import socket
import pathlib
import threading

# Ajustem els threads interns d'OpenCV a 4 just després d'importar-lo
cv2.setNumThreads(4)

# ==========================================
# 1. CLASSE DE LECTURA DE VÍDEO (FIL SECUNDARI)
# ==========================================
# Aquesta classe crea un fil invisible (Thread) que treballa en segon pla.
# La seva funció és llegir imatges de la càmera a màxima velocitat i descartar 
# les antigues, assegurant que la IA sempre rebi la imatge més fresca (0 lag).
class LectorTempsReal:
    def __init__(self, ruta_o_id):
        self.cap = cv2.VideoCapture(ruta_o_id)
        self.es_video = not isinstance(ruta_o_id, int)
        
        if not self.es_video:
            # ÉS LA CÀMERA: Demanem al maquinari que la imatge ja vingui reduïda (640x480).
            # Això allibera el 80% de l'amplada de banda del cable USB i CPU.
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
            self.espera = 0.0015 # Retard pràcticament nul
        else:
            # ÉS UN VÍDEO: Calculem els seus FPS originals per no reproduir-lo a càmera ràpida.
            self.fps_original = self.cap.get(cv2.CAP_PROP_FPS)
            self.espera = 1.0 / self.fps_original if self.fps_original > 0 else 0.03

        self.ret, self.frame = self.cap.read()
        self.actiu = True
        
        # Iniciem el fil invisible que netejarà el buffer infinitament
        self.fil = threading.Thread(target=self._bucle_lectura, daemon=True)
        self.fil.start()

    def _bucle_lectura(self):
        while self.actiu:
            ret, frame = self.cap.read()
            if ret:
                self.ret = ret
                # OPTIMITZACIÓ: Si processem un vídeo HD, l'encongim aquí al fil secundari
                # utilitzant INTER_LINEAR (més ràpid per la CPU) perquè no penalitzi la IA.
                if frame.shape[1] > 640:
                    self.frame = cv2.resize(frame, (640, int(640 * (frame.shape[0] / frame.shape[1]))), interpolation=cv2.INTER_LINEAR)
                else:
                    self.frame = frame
            else:
                self.actiu = False
            
            if self.es_video:
                time.sleep(self.espera)

    def read(self):
        # Quan YOLO demana un frame principal, retornem una còpia de l'últim guardat
        if self.frame is None: return self.ret, None
        return self.ret, self.frame.copy()

    def release(self):
        # Destruïm el fil i alliberem la càmera
        self.actiu = False
        self.cap.release()

    def isOpened(self):
        return self.actiu

# --- PARATXE DE COMPATIBILITAT WINDOWS/LINUX ---
# Evita que els models exportats en Windows (Pathlib) donin error a la Pi (Posix)
sys.modules['pathlib._local'] = pathlib
pathlib.WindowsPath = pathlib.PosixPath

# ==========================================
# 2. CONFIGURACIÓ UDP I VARIABLES GLOBALS
# ==========================================
# Inicialitzem l'emissor de dades cap al node de ROS o la placa ESP32
sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IP_PORT_ROS = ("127.0.0.1", 9999)

# Variables de la geometria del circuit (s'omplen llegint el JSON)
dades_pistes = {}
zones_perill_estatiques = []
mascares_pistes = {}
mapa_colors_pistes = {}
colors_pistes = [(255, 0, 0), (0, 255, 0), (0, 255, 255), (255, 0, 255), (0, 165, 255)]
escalat_fet = False  

mascara_mestra_pistes = None
mascara_zones_perill = None

# --- PARÀMETRES DE TRACKING I PREDICCIÓ ---
distancia_maxima = 250     # Píxels màxims per emparellar un cotxe d'un frame al següent
historial_maxim = 5        # Fotogrames que recordem del passat per calcular la trajectòria
paciencia_frames = 5       # Fotogrames de cortesia abans d'esborrar un cotxe que ha "desaparegut"
escala_prediccio = 5       # Multiplicador del vector direccional (cap a on mira el cotxe)
factor_amplada = 0.6       # Amplada del vector de predicció
llindar_moviment = 1.0     # Píxels de desplaçament mínims per considerar que està en marxa

# --- PARÀMETRES DE FRENADA AGRESSIVA I PRIORITAT ---
frames_minims_alerta = 1   # Dispara l'alerta immediatament (en 1 frame) per màxima reactivitat
v_maxima = 500             # Velocitat de PWM màxima a la recta
D_max = 250.0              # Distància d'avís (comença a aixecar el peu per frenar progressivament)
X_seguretat = 100.0        # Distància d'impacte (clava els frens a zero si entra en aquest marge)

memoria_colisions = {} 
objectes_actius = {}
proper_id = 1
memoria_velocitats = {}

# ==========================================
# 3. LÒGICA DE PROCESSAMENT PER FRAME
# ==========================================
# Funció principal que executa tota la visió artificial per cada imatge que arriba
def processar_frame(frame, temps_t, model, mostrar_pantalla, mida_yolo):
    global proper_id, memoria_colisions, mascares_pistes, zones_perill_estatiques
    global escalat_fet, mascara_mestra_pistes, mascara_zones_perill
    
    # ------------------------------------------------------
    # A) ESCALAT ADAPTATIU DEL JSON (S'executa 1 sol cop)
    # ------------------------------------------------------
    if not escalat_fet:
        h_img, w_img = frame.shape[:2]
        escala_x = w_img / 640  # Mapeja les coordenades guardades a la realitat actual
        escala_y = h_img / 480
        
        mascara_mestra_pistes = np.zeros((h_img, w_img), dtype=np.uint8)
        marge_ia = 100 # Engreixem les pistes per tapar només el fons inútil
        
        if dades_pistes:
            for idx, (nom_p, info_pista) in enumerate(dades_pistes.items()):
                mascara_temp = np.zeros((h_img, w_img), dtype=np.uint8)
                for segment in info_pista["segments"]:
                    seg_escalat = [[int(p[0] * escala_x), int(p[1] * escala_y)] for p in segment]
                    
                    gruix_visual = max(1, int(info_pista["gruix"] * escala_x))
                    cv2.polylines(mascara_temp, [np.array(seg_escalat)], False, 255, gruix_visual)
                    
                    gruix_ia_escalat = max(1, int((info_pista["gruix"] + marge_ia) * escala_x))
                    cv2.polylines(mascara_mestra_pistes, [np.array(seg_escalat)], False, 255, gruix_ia_escalat)
                    
                mascares_pistes[nom_p] = mascara_temp
                mapa_colors_pistes[nom_p.replace("_", " ").capitalize()] = colors_pistes[idx % len(colors_pistes)]
                
        if len(zones_perill_estatiques) > 0:
            zones_escalades = []
            for poligon in zones_perill_estatiques:
                poligon_escalat = np.zeros_like(poligon)
                poligon_escalat[:, 0] = (poligon[:, 0] * escala_x).astype(np.int32)
                poligon_escalat[:, 1] = (poligon[:, 1] * escala_y).astype(np.int32)
                zones_escalades.append(poligon_escalat)
            zones_perill_estatiques = zones_escalades
            
        mascara_zones_perill = np.zeros((h_img, w_img), dtype=np.uint8)
        if len(zones_perill_estatiques) > 0:
            for poligon in zones_perill_estatiques:
                cv2.fillPoly(mascara_zones_perill, [poligon], 255)
                
        escalat_fet = True

    # ------------------------------------------------------
    # B) INFERÈNCIA YOLO I APAGAT DE FONS
    # ------------------------------------------------------
    # Superposem la màscara mestra. Tot el que no sigui pista serà negre. 
    # Això facilita moltíssim la feina a la xarxa neuronal.
    if mascara_mestra_pistes is not None:
        frame_per_ia = cv2.bitwise_and(frame, frame, mask=mascara_mestra_pistes)
    else:
        frame_per_ia = frame

    resultats = model(frame_per_ia, imgsz=mida_yolo, conf=0.4, half=True, verbose=False)
    nous_centres_cotxes = []
    boxes = resultats[0].boxes
    
    # Extraiem els tensors matemàtics i n'aïllem les coordenades físiques
    if len(boxes) > 0:
        classes = boxes.cls.cpu().numpy()
        xywh_array = boxes.xywh.cpu().numpy()
        
        for i in range(len(boxes)):
            class_id = int(classes[i])
            if model.names[class_id] == 'car': 
                x, y, w, h = xywh_array[i]
                nous_centres_cotxes.append((x, y, w, h))

    # Esborrem cotxes que hagin sortit de la càmera (allibera RAM)
    ids_esborrar = [obj_id for obj_id, d_obj in objectes_actius.items() if (temps_t - d_obj['ultim_vist']) > paciencia_frames]
    for obj_id in ids_esborrar: del objectes_actius[obj_id]

    if mostrar_pantalla and len(zones_perill_estatiques) > 0:
        for poligon in zones_perill_estatiques:
            cv2.polylines(frame, [poligon], True, (0, 0, 255), 2)

    # ------------------------------------------------------
    # C) TRACKING (Identificació persistent)
    # ------------------------------------------------------
    # Vincula les deteccions actuals amb les passades comparant les distàncies 
    # per saber si un cotxe és el mateix del fotograma anterior.
    possibles_emparellaments = []
    for index_nou, d_cotxe in enumerate(nous_centres_cotxes):
        x_c, y_c, w_c, h_c = d_cotxe
        for obj_id, dades_obj in objectes_actius.items():
            h_obj = dades_obj['historial']
            pred_x = h_obj[-1][1] + (h_obj[-1][1] - h_obj[-2][1]) if len(h_obj) >= 2 else h_obj[-1][1]
            pred_y = h_obj[-1][2] + (h_obj[-1][2] - h_obj[-2][2]) if len(h_obj) >= 2 else h_obj[-1][2]
            
            # Utilitzem matemàtica al quadrat per no fer arrels innecessàries (optimització)
            dist_sq = (pred_x - x_c)**2 + (pred_y - y_c)**2
            if dist_sq < (distancia_maxima ** 2): possibles_emparellaments.append((dist_sq, obj_id, index_nou))

    possibles_emparellaments.sort(key=lambda x: x[0])
    ids_assignats = set(); deteccions_assignades = set(); objectes_actualitzats = []

    for dist, obj_id, index_nou in possibles_emparellaments:
        if obj_id not in ids_assignats and index_nou not in deteccions_assignades:
            ids_assignats.add(obj_id); deteccions_assignades.add(index_nou); objectes_actualitzats.append(obj_id)
            x_c, y_c, w_c, h_c = nous_centres_cotxes[index_nou]
            objectes_actius[obj_id]['historial'].append((temps_t, x_c, y_c, min(w_c, h_c), w_c, h_c))
            objectes_actius[obj_id]['ultim_vist'] = temps_t
            if len(objectes_actius[obj_id]['historial']) > historial_maxim: objectes_actius[obj_id]['historial'].pop(0)

    for index_nou, d_cotxe in enumerate(nous_centres_cotxes):
        if index_nou not in deteccions_assignades:
            x_c, y_c, w_c, h_c = d_cotxe
            objectes_actius[proper_id] = {'historial': [(temps_t, x_c, y_c, min(w_c, h_c), w_c, h_c)], 'ultim_vist': temps_t, 'pista_guardada': 'Fora'}
            objectes_actualitzats.append(proper_id); proper_id += 1

    # ------------------------------------------------------
    # D) CONSTRUCCIÓ DE POLÍGONS MATEMÀTICS
    # ------------------------------------------------------
    poligons_prediccio_cotxes = {}
    estat_vel = {obj_id: {'xoc': 'No', 'vel': v_maxima} for obj_id in objectes_actualitzats}

    for obj_id in objectes_actualitzats:
        historial = objectes_actius[obj_id]['historial']
        p_rec = objectes_actius[obj_id].get('pista_guardada', 'Fora')
        _, cx, cy, w_ver, w_orig, h_orig = historial[-1]

        # Comprovem la posició sobre les màscares mestres
        dins_zona_conflictiva = False
        if mascara_zones_perill is not None and cy < mascara_zones_perill.shape[0] and cx < mascara_zones_perill.shape[1]:
            dins_zona_conflictiva = (mascara_zones_perill[int(cy), int(cx)] == 255)
        
        pistes_trepitjades = [nom_p.replace("_", " ").capitalize() for nom_p, m in mascares_pistes.items() if cy < m.shape[0] and cx < m.shape[1] and m[int(cy), int(cx)] == 255]

        if dins_zona_conflictiva and p_rec != "Fora": nom_pista = p_rec
        elif pistes_trepitjades: nom_pista = p_rec if p_rec in pistes_trepitjades else pistes_trepitjades[0]
        else: nom_pista = "Fora"
        objectes_actius[obj_id]['pista_guardada'] = nom_pista

        color_pista = mapa_colors_pistes.get(nom_pista, (255,255,255))

        if len(historial) >= 3:
            t_v, x_v, y_v, _, _, _ = zip(*historial)
            
            dt = t_v[-1] - t_v[0]
            if dt > 0:
                vx = (x_v[-1] - x_v[0]) / dt
                vy = (y_v[-1] - y_v[0]) / dt
            else:
                vx, vy = 0, 0
                
            if math.sqrt(vx**2 + vy**2) > llindar_moviment:
                # COTXE EN MOVIMENT: Dibuixem un vector ("banya") que anticipa on estarà
                modul = math.sqrt(vx**2 + vy**2)
                nx = -vy / modul; ny = vx / modul; mw = (w_ver * factor_amplada) / 2
                pts = np.array([[cx+nx*mw, cy+ny*mw], [cx-nx*mw, cy-ny*mw], [cx+vx*escala_prediccio-nx*mw, cy+vy*escala_prediccio-ny*mw], [cx+vx*escala_prediccio+nx*mw, cy+vy*escala_prediccio+ny*mw]], dtype=np.float32)
            else:
                # COTXE ATURAT (NOVA LÒGICA): Creem un hitbox inflat (factor 1.3x) 
                # per evitar que altres cotxes s'apropin massa i provoquin friccions
                factor_escala = 1.3
                w_caixa = w_orig * factor_escala
                h_caixa = h_orig * factor_escala
                
                pts = np.array([
                    [cx - w_caixa/2, cy - h_caixa/2], 
                    [cx + w_caixa/2, cy - h_caixa/2], 
                    [cx + w_caixa/2, cy + h_caixa/2], 
                    [cx - w_caixa/2, cy + h_caixa/2]
                ], dtype=np.float32)
            
            poly = cv2.convexHull(pts)
            poligons_prediccio_cotxes[obj_id] = poly
            
            if mostrar_pantalla:
                cv2.polylines(frame, [np.int32(poly)], True, color_pista, 2)
                cv2.putText(frame, f"ID:{obj_id} ({nom_pista})", (int(cx), int(cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_pista, 2)

    # ------------------------------------------------------
    # E) DETECCIÓ DE COL·LISIONS I PRIORITAT DE PAS (OPTIMITZAT)
    # ------------------------------------------------------
    colisions_frame = {}
    
    # Pre-càlculs constants per evitar ofegar el bucle de frenada
    D_max_sq = D_max ** 2
    X_seg_sq = X_seguretat ** 2
    diff_D_X = D_max - X_seguretat
    
    for id_a, id_b in itertools.combinations(poligons_prediccio_cotxes.keys(), 2):
        
        # Filtre Espacial: Si els cotxes estan a l'altra punta del circuit (>400px), 
        # ignorem l'anàlisi de matrius per estalviar un 60% de CPU inútil.
        xa, ya = objectes_actius[id_a]['historial'][-1][1:3]
        xb, yb = objectes_actius[id_b]['historial'][-1][1:3]
        if (xa - xb)**2 + (ya - yb)**2 > 160000:
            continue

        intersecten, area_interseccio = cv2.intersectConvexConvex(poligons_prediccio_cotxes[id_a], poligons_prediccio_cotxes[id_b])
        
        if intersecten > 0:
            cx_xoc = int(np.mean(area_interseccio[:, 0, 0]))
            cy_xoc = int(np.mean(area_interseccio[:, 0, 1]))
            
            # Ens assegurem que el xoc té lloc en un punt delicat de la pista
            en_zona = False
            if mascara_zones_perill is not None and 0 <= cy_xoc < mascara_zones_perill.shape[0] and 0 <= cx_xoc < mascara_zones_perill.shape[1]:
                en_zona = (mascara_zones_perill[cy_xoc, cx_xoc] == 255)

            if en_zona:
                parella = (min(id_a, id_b), max(id_a, id_b))
                colisions_frame[parella] = {'frames': memoria_colisions.get(parella, {}).get('frames', 0) + 1, 'punt': (cx_xoc, cy_xoc)}
                
    memoria_colisions = colisions_frame
    
    for parella, d_col in memoria_colisions.items():
        if d_col['frames'] >= frames_minims_alerta:
            px, py = d_col['punt']
            
            if mostrar_pantalla:
                cv2.circle(frame, (px, py), 30, (0,0,255), 4)
                cv2.putText(frame, "!!! ALERTA !!!", (px-50, py-40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 2)
                
            id_a, id_b = parella
            if id_a in estat_vel and id_b in estat_vel:
                xa, ya = objectes_actius[id_a]['historial'][-1][1:3]
                xb, yb = objectes_actius[id_b]['historial'][-1][1:3]
                
                # Calculem la distància de cadascun fins a la cruïlla per aplicar la Prioritat
                dist_sq_a = (xa - px)**2 + (ya - py)**2
                dist_sq_b = (xb - px)**2 + (yb - py)**2
                
                # PRIORITAT DE PAS: El cotxe que està més a prop del xoc accelera per passar primer.
                # El cotxe que està més lluny (perdedor) aplica el fre per cedir el pas.
                if dist_sq_a > dist_sq_b:
                    id_frena, dist_sq_frena = id_a, dist_sq_a
                else:
                    id_frena, dist_sq_frena = id_b, dist_sq_b
                
                # FÓRMULA DE FRENADA MATEMÀTICA (Interpolació lineal sobre el cotxe perdedor)
                # 1. Menys de 100px -> Frena completament
                if dist_sq_frena <= X_seg_sq:
                    v_nova = 0  
                # 2. Més de 250px -> Manté velocitat base
                elif dist_sq_frena >= D_max_sq:
                    v_nova = v_maxima 
                # 3. Intermedi -> Reducció en rampa progressiva
                else:
                    v_nova = v_maxima * ((math.sqrt(dist_sq_frena) - X_seguretat) / diff_D_X)
                    
                estat_vel[id_frena]['vel'] = min(estat_vel[id_frena]['vel'], int(max(0, min(v_maxima, v_nova))))

    # ------------------------------------------------------
    # F) ENVIAMENT UDP A ROS/ESP32
    # ------------------------------------------------------
    for obj_id in objectes_actualitzats:
        pista_act = objectes_actius[obj_id].get('pista_guardada', 'Fora')
        vel_final = estat_vel[obj_id]['vel']
        
        if pista_act != 'Fora':
            # Només enviem el paquet UDP si hem de frenar o si estem recuperant velocitat
            # Estalvia ofegar l'amplada de banda amb enviaments innecessaris.
            if vel_final != v_maxima or memoria_velocitats.get(obj_id) != v_maxima:
                numero_pista = pista_act.replace("Pista ", "").strip()
                if numero_pista.isdigit():
                    missatge = f"{numero_pista},{vel_final}".encode('utf-8')
                    sock_udp.sendto(missatge, IP_PORT_ROS)
                memoria_velocitats[obj_id] = vel_final

    return frame

# ==========================================
# 4. MENÚ COMPLET D'ARRANCADA
# ==========================================
if __name__ == '__main__':
    # Inicialitza llegint el JSON dibuixat al laboratori
    if os.path.exists('config_pistes.json'):
        with open('config_pistes.json', 'r') as f:
            config = json.load(f)
            
            if "pistes" in config:
                dades_pistes = config["pistes"]
                zones_crues = config.get("zones_perill", [])
                zones_perill_estatiques = [np.array(z, np.int32) for z in zones_crues]
            else:
                dades_pistes = config
                zones_perill_estatiques = []
                
        print(f"✅ S'han carregat {len(zones_perill_estatiques)} zones de perill manuals.")
        
    print("\n" + "="*50)
    print(" 🚀 VISIÓ YOLO + UDP - TFG")
    print("="*50)
    print("1. Analitzar CÀMERA en temps real")
    print("2. Analitzar VÍDEO (.mp4)")
    
    opcio = input("\nTria una opció: ")

    if opcio in ['1', '2']:
        
        print("\n⚡ OPCIÓ DE RENDIMENT (Mida de la IA):")
        print("1 = Ultra-Ràpid (256px) -> Màxims FPS per a Raspberry Pi")
        print("2 = Equilibrat (320px) -> Bona relació FPS/Precisió")
        print("3 = Precís (640px) -> Ideal per a ordinadors potents (Lent a la Pi)")
        opcio_mida = input("Tria una opció (Enter per Equilibrat): ").strip()
        
        if opcio_mida == '1':
            mida_yolo = 256
        elif opcio_mida == '3':
            mida_yolo = 640
        else:
            mida_yolo = 320 
        
        print(f"\nCarregant el model YOLO (a {mida_yolo}px)...")
        # task='detect' elimina el WARNING del terminal per defecte de YOLO
        model = YOLO('best_n_26_256_only_car_ncnn_model', task='detect') 

        salt_frames = 1
        if opcio == '2':
            ruta_video = input("\nNom del vídeo (ex: video.mp4): ")
            cap = LectorTempsReal(ruta_video)
            
            print("\n⚙️ OPCIÓ DE VÍDEO:")
            print("El vídeo es reproduirà a temps real 1x, i YOLO agafarà sempre l'últim frame.")
            salt_frames = 1
        else:
            cap = LectorTempsReal(0)

        resposta = input("\nVols veure les imatges amb els vectors per pantalla (pop-up)? (S/N): ").strip().upper()
        mostrar_pantalla = (resposta == 'S')
        
        if mostrar_pantalla:
            print("\n👀 Mode Visual activat (Més lent). Prem 'q' a la finestra emergent per aturar.")
        else:
            print("\n⚡ Mode Cec activat (Màxim rendiment). Prem Ctrl+C a la consola per aturar.")

        temps_t = 0
        frames_processats = 0
        temps_processament_total = 0.0
        frames_reals_analitzats = 0
        
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret: 
                    print("\nS'ha arribat al final.")
                    break
                
                temps_t += 1
                
                if opcio == '2' and temps_t % salt_frames != 0:
                    continue
                
                # El processament central
                t_inici = time.time()
                frame_final = processar_frame(frame, temps_t, model, mostrar_pantalla, mida_yolo)
                t_fi = time.time()
                
                # Matemàtiques per calcular la taxa de FPS purs a la terminal
                temps_invertit = t_fi - t_inici
                temps_processament_total += temps_invertit
                frames_reals_analitzats += 1
                frames_processats += 1
                fps = 1.0 / temps_invertit if temps_invertit > 0 else 0
                
                if mostrar_pantalla:
                    cv2.putText(frame_final, f"FPS: {fps:.1f} (IA: {mida_yolo}px)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow('Deteccio TFG', frame_final)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break
                else:
                    print(f"-> Analitzant Frame: {temps_t} | Velocitat: {fps:.1f} FPS", end='\r')
                
        except KeyboardInterrupt:
            print("\n\n🛑 Procés aturat manualment per l'usuari.")
            
        finally:
            # Apagat net de perifèrics
            cap.release()
            cv2.destroyAllWindows()
            
            if frames_reals_analitzats > 0:
                fps_mitjans = frames_reals_analitzats / temps_processament_total
                print("\n" + "="*40)
                print(" 📊 RESUM ANALÍTIC DEL RENDIMENT")
                print("="*40)
                print(f"Total frames analitzats : {frames_reals_analitzats}")
                print(f"FPS Mitjans (Reals)     : {fps_mitjans:.2f} FPS")
                print("="*40 + "\n")