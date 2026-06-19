# Sistema de visió artificial i control anticol·lisió per a un circuit d'Scalextric

**Codis annexos del Treball de Fi de Grau (TFG)**

Aquest repositori conté el codi font del sistema desenvolupat per al TFG. L'objectiu del
projecte és **detectar en temps real els cotxes d'un circuit d'Scalextric mitjançant visió
artificial (YOLO), predir-ne la trajectòria, detectar possibles col·lisions a les zones de
perill i controlar la velocitat de cada cotxe (via ROS2) per evitar que xoquin.**

Tot el codi està **escrit i comentat en català**, pensat perquè es pugui llegir i entendre
sense executar-lo.

---

## ⭐ Com llegir aquest repositori

El projecte es va desenvolupar **per etapes**. Els programes `Detectar_vectors` i
`Dibuixar_Vectors` són els **passos previs de desenvolupament**: cadascun resol una part del
problema per separat (primer detectar, després analitzar trajectòries i col·lisions).

Un cop validades aquestes parts, **tota la lògica s'unifica en un únic programa que funciona
en temps real**: [`visio_udp_V3_V10_ncnn_n_26_256_Commentat - copia.py`](visio_udp_V3_V10_ncnn_n_26_256_Commentat%20-%20copia.py).

```
   ETAPES PRÈVIES (desenvolupament)              PROGRAMA FINAL UNIFICAT
   ─────────────────────────────────            ───────────────────────
   Detectar_vectors_4.py  ─┐
   (detecció YOLO + .txt)   │
                            ├──►  visio_udp_V3_V10_..._Commentat.py
   Dibuixar_Vectors_6.py  ─┘      (detecció + tracking + predicció +
   (tracking, vectors,            col·lisions + control, TOT EN TEMPS REAL)
    col·lisions des de .txt)
```

> En resum: **`Detectar_vectors` + `Dibuixar_Vectors` són la versió "per passos" (treballen
> amb fitxers intermedis), i `visio_udp_...` és la versió final que ho fa tot junt, en directe
> i enviant les ordres de velocitat al circuit.**

---

## 📂 Descripció dels fitxers

| Fitxer | Etapa | Què fa |
|--------|-------|--------|
| [`Dibuixar_pistes_V3.py`](Dibuixar_pistes_V3.py) | Calibratge (previ) | Eina de calibratge. Fa una foto del circuit i permet dibuixar a mà, amb el ratolí, les **pistes** i les **zones de perill**. Desa la geometria a `config_pistes.json`. S'executa **una sola vegada** abans de tot. |
| [`Detectar_vectors_4.py`](Detectar_vectors_4.py) | **Pas previ 1** | El **detector**. Passa el model YOLOv5 per càmera, carpeta d'imatges o vídeo i, per cada fotograma, desa la imatge amb les caixes i un `.txt` amb les coordenades dels cotxes detectats. |
| [`Dibuixar_Vectors_6.py`](Dibuixar_Vectors_6.py) | **Pas previ 2** | L'**analitzador de trajectòries**. Llegeix els `.txt` del detector i fa la "intel·ligència": seguiment (tracking) de cada cotxe amb ID estable, vector de predicció, detecció de col·lisions a les zones de perill i càlcul de la frenada (telemetria a CSV). |
| [`visio_udp_V3_V10_ncnn_n_26_256_Commentat - copia.py`](visio_udp_V3_V10_ncnn_n_26_256_Commentat%20-%20copia.py) | **Programa final unificat** | Unifica tot l'anterior en **un sol programa en temps real**: captura la càmera, detecta (YOLO/NCNN optimitzat per a Raspberry Pi), fa tracking, prediu trajectòries, detecta col·lisions i **envia per UDP les ordres de velocitat** de cada pista. |
| [`control_udp.py`](control_udp.py) | Pont de control | Node **ROS2** que fa de pont: escolta les ordres UDP que envia el programa de visió i les republica dins de ROS2 (`Int32MultiArray`), que és qui controla físicament els cotxes. |

---

## 🔧 Tecnologies utilitzades

- **Python 3**
- **YOLO26 / Ultralytics YOLO** — detecció d'objectes (cotxes)
- **NCNN** — inferència optimitzada per a Raspberry Pi (versió final)
- **OpenCV** — captura i tractament d'imatge
- **NumPy** — càlcul de vectors i geometria
- **ROS2** (`rclpy`) — control dels cotxes
- **Sockets UDP** — comunicació entre el programa de visió i ROS2

---

## ▶️ Flux d'execució del sistema final

1. **Calibratge** (un sol cop): `Dibuixar_pistes_V3.py` → genera `config_pistes.json`.
2. **Visió en temps real**: `visio_udp_V3_V10_...py` → detecta, analitza i envia ordres per UDP.
3. **Control**: `control_udp.py` (node ROS2) → rep les ordres i mou els cotxes.

> Els programes `Detectar_vectors_4.py` i `Dibuixar_Vectors_6.py` **no** calen per al
> funcionament final: són les etapes de desenvolupament que es van unificar dins de
> `visio_udp_...`. S'inclouen al repositori perquè documenten el procés de construcció del TFG.

---

## ℹ️ Notes

- Els fitxers de model entrenat (`best.pt`, `*.ncnn`...), les imatges de sortida i els fitxers
  generats en execució **no** s'inclouen al repositori (vegeu [`.gitignore`](.gitignore)), ja que
  són dades pesades o generades automàticament i no formen part del codi font.
- Tot el codi està àmpliament comentat en català per facilitar-ne la revisió.
