# Compte rendu séance 1 — Arthur DOMMANGE

Projet drone VTOL tilt-rotor (type V-22), Rob3 Polytech Nice Sophia, binôme avec Petanky SEKONGO.
Séance du 6 novembre 2026, encadrée par M. LEBRETON.

## Objectif de la séance

Pour cette première séance on n'avait pas encore toutes les pièces. De mon côté j'avais deux choses à faire : l'inventaire de ce qu'on a reçu, et la CAO de la nacelle sur Fusion 360 à partir des cotes des pièces.

## 1. Inventaire des pièces

J'ai comparé ce qu'on a reçu avec la liste du matériel du rapport bibliographique (tableau III.2).

- **Reçu (6)** : les 2 ESC 40 A, les 3 servos KST DS215MG, le capteur de vitesse air ASPD-4525 avec son tube de Pitot, le filament PETG du fablab, 3 paires d'hélices Gemfan 1045 (10 × 4,5, une paire de rechange) et un lot de 10 roulements MR105ZZ.
- **Reçu mais à vérifier (4)** :
  - les servos MG90S : j'en ai vu 4 sur la photo, il faut recompter les sachets ;
  - la radio : vérifier que c'est bien la FS-i6X et que le récepteur FS-iA6B est dans la boîte ;
  - l'adaptateur de niveau 3,3 / 5 V : c'est un TXS0108E, qui marche souvent mal en I²C, à tester avec le capteur ;
  - les engrenages : 2 reçus sur les 6 prévus (tableau III.2), il faut vérifier que ce sont bien une roue de 40 dents et un pignon de 30 dents, le module et l'alésage.
- **Pas encore reçu (9)** : entre autres la carte Arduino Nano 33 BLE, les moteurs X2216, les batteries, l'UBEC et les tubes carbone.
- **Hors rapport, pas encore reçu (2)** : un châssis carbone et des plaques GRP pour l'armature. Ils ont été ajoutés après le rapport.

Comme la carte et les moteurs ne sont pas encore là, on ne peut rien tester en vrai pour l'instant. J'ai donc fait toute la nacelle en CAO pour être prêt quand les pièces arriveront.

## 2. CAO de la nacelle

### Ce qui a changé par rapport au rapport

Dans le rapport, la nacelle tournait sur un palier fixé au bout de l'aile, avec le servo fixé sur l'aile. En regardant une vidéo de maquette de V-22, j'ai changé de principe :

- la nacelle est faite de **deux flasques en FR4 de 2 mm**, avec des triangles évidés pour l'alléger ;
- **le servo KST est dans la nacelle**, en haut, vissé sur une platine en PETG ;
- son **pignon de 30 dents roule sur un secteur denté fixe de 40 dents**, monté sur l'axe Ø5 ;
- l'axe Ø5 est collé dans le bout du longeron Ø10 avec une bague en PETG. Il n'y a plus de palier en bout d'aile ;
- la nacelle tourne sur deux roulements MR105 (un par flasque), dans des moyeux en PETG ;
- le moteur X2216 est vissé sur un support en PETG en haut des flasques (4 trous M3, entraxes 16 et 19 mm).

On garde le même rapport de réduction (4:3) et le même entraxe (35 mm) que dans le rapport. Le secteur denté sert aussi de butée : la nacelle ne peut aller que de 0° (hélicoptère) à 90° (avion).

### Comment je l'ai fait

Le modèle est généré par un script Python dans Fusion. Toutes les cotes sont des paramètres en haut du fichier, ce qui permet de tout reconstruire quand on change une valeur (par exemple quand on aura mesuré les vraies pièces). Il y a un script pour le drone complet et un autre pour la nacelle seule.

### Problèmes rencontrés

- Au début, toutes les pièces étaient grises : le script cherchait les couleurs par leur nom anglais alors que Fusion est en français. Corrigé.
- Les engrenages étaient de simples cylindres sans dents. Je les ai refaits avec un vrai profil de denture.
- Ma première version avait encore un boîtier sous la nacelle (l'ancien palier). Je l'ai enlevé puisque le seul servo est maintenant en haut.

### Vérifications

- Les dents du pignon et du secteur ne se touchent pas de 0° à 90° (vérifié à 0, 17, 30, 45, 60 et 90°).
- L'axe du moteur passe par l'axe de basculement, donc pas de couple dû à la poussée sur le servo.
- En mode avion, il y a 48 mm entre l'hélice et le bord d'attaque (20 mm avec l'ancienne nacelle).

### Fichiers pour la fabrication

Le script de la nacelle sort directement :

- les STL des pièces en PETG (support moteur et platine, moyeux, bague, secteur, pignon d'essai) ;
- le DXF des flasques (34 × 82 mm, 4 à faire dans les plaques FR4, en fraisage) ;
- deux éprouvettes : une avec 4 trous de roulement de tailles différentes pour régler le jeu avec l'imprimante, et une pour vérifier l'entraxe de 35 mm des engrenages.

## 3. Écarts avec le rapport bibliographique

- Le palier de bout d'aile n'existe plus et le servo est dans la nacelle (à changer dans le III.3 et le tableau III.4).
- La course passe de −5°/+100° à 0°/90°.
- La nacelle est plus lourde avec le servo dedans, donc son inertie sera plus grande que les 8·10⁻⁵ kg·m² estimés.
- L'envergure hors hélices fait 1 146 mm, alors que le cahier des charges demande 1 100 mm maximum. On doit en parler avec mon binôme.

## 4. Pour la prochaine séance

- Imprimer les éprouvettes, essayer un roulement MR105 dans les logements, puis imprimer les pièces de la nacelle.
- Compter les dents des engrenages reçus et vérifier qu'ils correspondent à la CAO.
- Mesurer un servo KST reçu (oreilles de fixation) et corriger la CAO.
- Recompter les MG90S et vérifier la radio.
- Commencer le banc de poussée.

**Remarques de M. LEBRETON :**
