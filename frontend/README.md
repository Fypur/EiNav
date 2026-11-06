# Robot Control — Interface web

Une page web (`live.html`) servie par le programme du robot, utilisable depuis le navigateur de tout appareil sur le même réseau. Elle n'a besoin que de Python, sans rien à installer pour l'interface.

## Fonctionnalités

- choix entre la livraison sur la grille et le suivi de ligne sur le parcours en 8, boutons Démarrer et Arrêter, état du robot, image traitée ;
- en livraison : choix du départ, de l'arrivée et de l'orientation, position sur la grille, flags « destination atteinte » et « retour au départ », arêtes bloquées ;
- à la fin d'une mission, le champ d'orientation prend l'orientation finale du robot ;
- panneau **Réglages** pour changer les constantes du robot pendant qu'il roule.

## Installation et lancement

Python 3.10+ recommandé. Le programme du robot a besoin de `numpy`, `opencv-python` et `pyserial`.

Sur le Raspberry Pi, depuis la racine du dépôt :

```bash
python EiNav.py
```

`EiNav.py` lance l'interface et affiche son adresse dans le terminal. Sur une machine avec écran, elle s'ouvre aussi dans le navigateur. **Le robot reste immobile** jusqu'au clic sur **Démarrer**.

Pour faire une livraison :

1. choisir le départ, l'arrivée et l'orientation ;
2. poser le robot sur le croisement de départ, dans l'orientation choisie (la caméra ne voit pas ce croisement : le robot s'y considère déjà arrivé) ;
3. cliquer sur **Démarrer**. Le robot va à l'arrivée, y fait un tour complet (l'interface affiche « Livraison en cours »), fait demi-tour, revient au départ et attend l'ordre suivant.

Pour suivre le parcours en 8 :

1. choisir **Suivi de ligne en 8** ;
2. poser le robot sur la ligne ;
3. cliquer sur **Démarrer**. Le robot suit la ligne et traverse le croisement tout droit, sans s'arrêter, jusqu'au clic sur **Arrêter**.

Dans les deux modes, si le robot en suivi de ligne ne voit plus aucune ligne pendant plus de `LINE_LOST_TIMEOUT` secondes (`robot/utils.py`), il s'arrête et l'interface affiche « Arrêt : ligne perdue ». Il repart seulement après un nouveau clic sur **Démarrer** : en livraison, il faut le reposer sur un croisement de départ.

Dans les deux modes, le capteur ultrason freine le robot devant un obstacle : il ralentit à moins de 60 cm et s'arrête complètement à 20 cm, distances mesurées depuis le capteur (`OBSTACLE_*` dans `robot/utils.py`). Il repart seul quand l'obstacle s'éloigne au-delà de 25 cm. Si le capteur ne donne aucune mesure, le robot l'ignore et poursuit sa mission **sans détecter les obstacles** ; l'interface affiche alors « Ultrason sans mesure : ignoré ». Le programme `arduino/src` doit être téléversé sur la carte, car il contient la commande de lecture du capteur.

En livraison, si le robot reste arrêté plus d'une seconde devant un obstacle (`OBSTACLE_CONFIRM_SECONDS`), l'arête est retirée du graphe et barrée de rouge sur la grille. Le robot fait demi-tour, revient à son dernier nœud à vitesse réduite et repart par un nouveau chemin. Cette vitesse et les durées des rotations faites à l'arrivée au nœud se règlent dans la section **Après un obstacle** du panneau **Réglages**. Sans chemin possible, il s'arrête et l'interface affiche « Arrêt : aucun chemin ». Les arêtes restent bloquées d'une mission à l'autre, jusqu'au bouton **Réinitialiser la carte**, disponible hors mission, ou au redémarrage du programme. Sur le parcours en 8, le robot attend simplement que la voie se libère.

Depuis un navigateur sur le même réseau :

| Adresse | Contenu |
| --- | --- |
| `http://IP_DU_PI:8001` | Commande du robot (livraison ou parcours en 8) et image traitée |
| `http://IP_DU_PI:8000` | Image traitée seule |

Le point d'accès mobile doit permettre la communication locale entre les appareils.

## Réglages du robot

Les valeurs par défaut sont dans le fichier `utils.py` de chaque dossier (`robot/`, `vision/`, `planning/`, `connections/`).

Les plus utiles se changent dans le panneau **Réglages** de la page : vitesses, kp, durée d'approche, virages, seuil de blanc, position des quatre côtés de la zone analysée (le rectangle bleu sur l'image).

Si cette zone fait moins de 30 % de la largeur de l'image (`INTERSECTION_MIN_WIDTH_RATIO`), les croisements ne peuvent plus être détectés. Un changement s'applique aussitôt, même en mission, et il est conservé dans `settings.json`, sur le Pi et hors de git.

Une valeur modifiée depuis l'interface prend le pas sur `utils.py`. Le bouton ↺ à côté d'un réglage le remet à la valeur de `utils.py`. Pour ajouter un réglage, compléter la liste `SETTINGS` de `robot/tuning.py`.

## Précautions

- **Arrêter** coupe les moteurs au tour de boucle suivant, à condition que le réseau fonctionne : ce n'est pas un arrêt d'urgence matériel. `Ctrl+C` sur le Pi arrête aussi le robot.
- Le bouton STOP de l'onglet simulation est **visuel uniquement** : il n'arrête pas le robot réel.
- L'application ne comporte pas d'authentification : **toute personne sur le réseau peut faire démarrer le robot**. Ne pas exposer les ports à Internet et préférer un réseau local fiable.
