# Robot Control — Interface Streamlit V2

Une interface web servie par le Raspberry Pi, utilisable depuis le navigateur de tout appareil sur le même réseau. L'onglet **Robot — Livraison** commande le robot réel et l'affiche en direct ; les autres onglets sont une **simulation** et n'envoient aucune commande au robot.

## Fonctionnalités

- **Robot — Livraison** : choix du départ, de l'arrivée et de l'orientation, boutons Démarrer et Arrêter, position sur la grille, état, flags « destination atteinte » et « retour au départ », image traitée. Cette page (`live.html`) est servie par le programme du robot ;
- **Maillage cartésien de 16 points (grille 4 × 4)**, identifiants A–P, avec axes X/Y, coordonnées initiales de 0 à 3, départ M = (0, 0) ;
- choix de destination en cliquant un point ou depuis une liste ; chemin le plus court calculé et visualisé en bleu (NetworkX) ;
- commandes **Préparer le trajet**, **Retour au départ**, **Avancer d'un point** et **STOP** : simulation uniquement ;
- mode indépendant **Suivi de ligne blanche**, qui accepte une **courbe non rectiligne** définie par des points de contrôle X/Y ;
- éditeur de la ligne blanche (courbe lisse Catmull-Rom ou segments droits), longueur totale, curseur de progression et avance virtuelle de 5 % ;
- édition des coordonnées des 16 nœuds et de leurs arêtes ; sauvegarde dans `carte.json` ;
- édition et sauvegarde de la ligne dans `ligne_blanche.json` ;
- panneau caméra optionnel : flux vidéo réel du robot, diffusé par `vision/camera_pi.py` (MJPEG, `http://IP_DU_PI:8000/stream.mjpg`) quand le programme du robot tourne ; journal des interactions.

## Installation et lancement

Python 3.10+ recommandé :

Sur le Raspberry Pi, depuis la racine du dépôt :

```bash
pip install -r frontend/requirements.txt
python EiNav.py
```

`EiNav.py` lance l'interface et affiche son adresse dans le terminal. Sur une machine avec écran, elle s'ouvre aussi dans le navigateur. **Le robot reste immobile** jusqu'au clic sur **Démarrer**.

Pour faire une livraison :

1. choisir le départ, l'arrivée et l'orientation ;
2. poser le robot sur la ligne, juste avant le point de départ, dans l'orientation choisie ;
3. cliquer sur **Démarrer**. Le robot va à l'arrivée, fait demi-tour, revient au départ et attend l'ordre suivant.

Depuis un navigateur sur le même réseau :

| Adresse | Contenu |
| --- | --- |
| `http://IP_DU_PI:8501` | Interface Streamlit complète |
| `http://IP_DU_PI:8001` | Mode livraison, grille en direct et image traitée, sans Streamlit |
| `http://IP_DU_PI:8000` | Image traitée seule |

La page du port 8001 ne dépend que de Python : elle fonctionne même si Streamlit n'est pas installé. Le point d'accès mobile doit permettre la communication locale entre les appareils.

Pour travailler sur l'interface sans robot : `cd frontend && streamlit run app.py` (l'onglet du robot affiche alors « Robot hors ligne »).

## Structure

| Fichier | Usage |
| --- | --- |
| `app.py` | Interface Streamlit et visualisations Plotly |
| `live.html` | Mode livraison, grille en direct et image traitée, servie par `robot/telemetry.py` |
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

- **Arrêter** coupe les moteurs au tour de boucle suivant, à condition que le réseau fonctionne : ce n'est pas un arrêt d'urgence matériel. `Ctrl+C` sur le Pi arrête aussi le robot.
- Le bouton STOP de l'onglet simulation est **visuel uniquement** : il n'arrête pas le robot réel.
- L'application ne comporte pas d'authentification : **toute personne sur le réseau peut faire démarrer le robot**. Ne pas exposer les ports à Internet et préférer un réseau local fiable.
