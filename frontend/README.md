# Robot Control — Interface Streamlit V2 (simulation)

Une interface web **uniquement en Python / Streamlit**, utilisable depuis le navigateur d'un téléphone connecté au même Wi-Fi que le Raspberry Pi. **Aucun Arduino ni capteur n'est connecté à ce stade**.

## Fonctionnalités

- **Maillage cartésien de 16 points (grille 4 × 4)**, identifiants A–P, avec axes X/Y, coordonnées initiales de 0 à 3, départ M = (0, 0) ;
- choix de destination en cliquant un point ou depuis une liste ; chemin le plus court calculé et visualisé en bleu (NetworkX) ;
- commandes **Préparer le trajet**, **Retour au départ**, **Avancer d'un point** et **STOP** : simulation uniquement ;
- mode indépendant **Suivi de ligne blanche**, qui accepte une **courbe non rectiligne** définie par des points de contrôle X/Y ;
- éditeur de la ligne blanche (courbe lisse Catmull-Rom ou segments droits), longueur totale, curseur de progression et avance virtuelle de 5 % ;
- édition des coordonnées des 16 nœuds et de leurs arêtes ; sauvegarde dans `carte.json` ;
- édition et sauvegarde de la ligne dans `ligne_blanche.json` ;
- panneau caméra optionnel : flux vidéo réel du robot, diffusé par `camera_pi.py` (MJPEG, `http://IP_DU_PI:8000/stream.mjpg`) quand le programme du robot tourne ; journal des interactions.

## Installation et lancement

Python 3.10+ recommandé :

```bash
cd robot_streamlit
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
streamlit run app.py --server.address 0.0.0.0 --server.port 8501
```

Sous Windows, remplacer `source .venv/bin/activate` par `.venv\Scripts\activate`.

Sur le Raspberry Pi connecté au partage de connexion de ton téléphone :

```bash
hostname -I
```

Puis depuis le navigateur du téléphone, ouvrir `http://ADRESSE_IP_DU_PI:8501` (avec l'adresse **réellement attribuée**). Le point d'accès mobile doit permettre la communication locale entre les appareils.

## Structure

| Fichier | Usage |
| --- | --- |
| `app.py` | Interface Streamlit et visualisations Plotly |
| `navigation.py` | Validation du maillage et recherche de chemin |
| `carte.json` | Grille de 16 points et 24 arêtes par défaut |
| `ligne.py` | Validation, interpolation et positionnement sur une ligne courbe |
| `ligne_blanche.json` | Points X/Y du tracé blanc et paramètre de lissage |
| `requirements.txt` | Bibliothèques nécessaires |

## Comprendre les deux modes

**Navigation sur maillage :** les 16 points constituent les lieux de décision. Chaque arête doit relier deux points accessibles ; le chemin est calculé sur le graphe non orienté avec ses distances.

**Suivi de ligne blanche :** la trajectoire est une ligne géométrique **indépendante des arêtes**. Elle peut tourner, serpenter ou décrire une courbe. La courbe Catmull-Rom passe par les points de contrôle ; le curseur fait avancer un point représentant le robot le long de cette courbe, à distance parcourue constante par pourcentage. **Aucun capteur réel n'est simulé** et rien ne déplace physiquement le robot. L'objectif est de valider l'interface.

Par défaut, tous les 16 nœuds sont présents ; l'éditeur permet de changer leurs coordonnées et leurs liaisons, mais pas d'en modifier le nombre.

## Précautions

- STOP est **visuel uniquement**, pas un arrêt d'urgence matériel.
- Aucun appel série, moteur, caméra réelle, ni automatisme physique.
- L'application ne comporte pas d'authentification : ne pas exposer le port à Internet et préférer un réseau local fiable.
