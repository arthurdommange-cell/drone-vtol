# Séance 1 — Rapport personnel

**Projet :** drone VTOL tilt-rotor bi-rotor (type V-22), robotique 3A, Polytech Nice Sophia
**Auteur :** Arthur DOMMANGE (binôme : Petanky Mamadou SEKONGO)
**Date :** 6 novembre 2026
**Séance encadrée par :** M. LEBRETON

---

## 1. Objectifs de la séance

D'après le planning, la séance 1 se fait sans pièces reçues. Mes objectifs étaient les suivants :

| Qui | Tâche | Livrable |
| --- | --- | --- |
| À deux | Vérifier les liens et les délais de livraison, passer la commande | Commande passée |
| Moi | CAO de la nacelle et du secteur denté à partir des cotes des fournisseurs (fixation du X2216, servo KST, roulements, pignon) | CAO nacelle v1 |

La partie de Petanky (schéma électrique, découpage du code) fait l'objet de son propre rapport.

## 2. Travail réalisé : CAO de la nacelle v1

Le modèle 3D du drone est généré dans Fusion par un script Python paramétrique (`DroneVTOL/DroneVTOL.py`). Chaque cote est un paramètre en tête de fichier : changer une valeur et relancer le script reconstruit tout l'assemblage, nacelles, aile, fuselage et empennage.

### 2.1 Nouvelle architecture de la nacelle

J'ai abandonné la nacelle du rapport bibliographique (berceau, palier de bout d'aile et servo fixé sur l'aile). Elle est remplacée par une nacelle **à deux flasques**, inspirée d'une maquette de V-22 :

- **deux flasques en FR4** (verre-époxy) de 2 mm, avec des allègements en triangles, reliés par deux entretoises M3 et par le support moteur ;
- **un seul servo KST DS215MG**, embarqué en haut de la nacelle et vissé par ses oreilles sur une platine en PETG ;
- son **pignon de 30 dents** roule sur un **secteur denté fixe de 40 dents** (roue POM recoupée), monté sur l'axe ;
- **l'axe Ø5 est fixe** : il est collé dans le bout du longeron carbone Ø10 grâce à une bague en PETG, et le palier de bout d'aile disparaît ;
- **deux roulements MR105ZZ** (5 × 10 × 4), un par flasque, logés dans des moyeux en PETG : la nacelle tourne autour de l'axe fixe ;
- le **moteur SunnySky X2216** est vissé sur le support en PETG (4 trous M3, entraxes 16 et 19 mm), avec son axe de poussée passant par l'axe de basculement.

Le principe retenu en I.3 est conservé : le servo ne transmet que le couple de basculement, et les roulements reprennent la poussée et le moment gyroscopique.

### 2.2 Cotes utilisées

| Pièce | Cote retenue | Origine |
| --- | --- | --- |
| Plaques FR4 | 200 × 200 × 2 mm | Tableau III.2 (commande) |
| Flasque | 34 × 82 × 2 mm, 4 par drone | CAO |
| Roulement MR105ZZ | 5 × 10 × 4 mm | Tableau III.2 |
| Engrenages | module 1, 40 et 30 dents, largeur 6 mm, entraxe 35 mm | Tableau III.2, I.3 |
| Servo KST DS215MG | boîtier 23,5 × 12 × 24,5 mm ; oreilles 32,5 mm, trous à 28 mm | à relever sur le servo reçu |
| Moteur X2216 | Ø28 × 26 mm, 4 × M3 sur 16 / 19 mm | à vérifier sur le moteur reçu |
| Longeron | tube carbone Ø10 × 8 mm | alésage à mesurer |
| Axe de basculement | tube carbone Ø5 | Tableau III.2 |

## 3. Choix de conception et vérifications

| Point vérifié | Résultat |
| --- | --- |
| Rapport et couple | 4:3 conservé : 4,1 kg·cm sur la nacelle (I.3) |
| Course | 0° (hélicoptère) à 90° (avion), limitée par le secteur denté et par des butées sur la liaison |
| Engrènement pignon / secteur | Aucune interférence à 0, 17, 30, 45, 60 et 90° (contrôle numérique des profils en développante) |
| Jeux internes | 1,0 mm entre entretoises et secteur, 1,3 mm entre entretoises et pignon, 6,5 mm entre pignon et support |
| Décalage poussée / axe de basculement | 0 mm (objectif < 5 mm, I.3) |
| Garde hélice / bord d'attaque en mode avion | 48 mm (20 mm avec l'ancienne nacelle) |
| Surface alaire | 0,210 m² (0,198 m² avant, objectif 0,22 m²), car l'aile va maintenant jusqu'à la nacelle |

Dans Fusion, la nacelle est reliée à l'axe par une liaison pivot avec butées. Le pignon a sa propre liaison pivot : avec un lien de mouvement (90° de nacelle pour 120° de pignon, en sens inverse), on peut animer le basculement complet.

## 4. Écarts avec le rapport bibliographique

- **§ III.3 et Tableau III.4 :** le palier de bout d'aile et le support de servo fixe n'existent plus. Ils sont remplacés par les flasques, la platine de servo et la bague longeron / axe.
- **Course :** 0 à 90° au lieu de −5 à +100°. Une course plus large reste possible en modifiant un seul paramètre (`COURSE_NACELLES`).
- **Masse et inertie de la nacelle :** le servo bascule maintenant avec la nacelle, donc l'inertie It sera supérieure à l'estimation de 8·10⁻⁵ kg·m² (II.2.2). Elle sera relevée dans Fusion (Propriétés physiques) une fois les matériaux affectés.
- **Envergure hors hélices :** 1 146 mm, au-dessus des 1 100 mm du cahier des charges (C2). Point à trancher avec mon binôme.

## 5. Bilan et préparation de la séance 2

**Livrables de la séance :**

- [x] CAO de la nacelle v1 (script `DroneVTOL.py`)
- [ ] Commande passée (à cocher)

**À faire en séance 2 :**

- [ ] Imprimer une nacelle v1 et un secteur denté pour vérifier l'assemblage et l'entraxe de 35 mm
- [ ] Imprimer l'éprouvette de logement de roulement, puis régler `JEU_LOGEMENT`
- [ ] Préparer le plan de découpe et de perçage des flasques dans les plaques FR4
- [ ] À réception : mesurer le servo (oreilles), le moteur et l'alésage du tube, puis mettre à jour les paramètres

**Remarques de M. LEBRETON :** _à compléter_
