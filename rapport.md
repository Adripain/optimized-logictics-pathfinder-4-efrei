# Rapport - Optimisation de tournees logistiques

## Objectif

Le but du projet est de simuler un petit systeme de logistique urbaine. On cherche a:

- calculer le plus court chemin entre deux points;
- construire une matrice de distances entre le depot et les points de livraison;
- produire une tournee passant par toutes les livraisons;
- repartir les livraisons entre plusieurs vehicules de capacite limitee.

Le programme est ecrit en Python pur, sans dependance externe. Le graphe utilise est un extrait simplifie de Paris avec des coordonnees reelles. Les distances des rues sont estimees avec la distance geographique de Haversine.

## Donnees

Le graphe contient 16 noeuds et 27 aretes non orientees. Le depot choisi est la Gare de Lyon. Les autres noeuds representent des lieux connus: Bastille, Republique, Louvre, Montparnasse, Nation, etc.

Dans un projet plus avance, on pourrait remplacer ces fichiers CSV par un vrai export OpenStreetMap, par exemple avec `osmnx`. Ici le choix d'un petit graphe rend le projet autonome et plus facile a presenter.

## Algorithmes de plus court chemin

### Dijkstra

Dijkstra calcule le plus court chemin depuis un noeud source vers les autres noeuds avec des poids positifs. Il maintient une distance provisoire pour chaque noeud et choisit toujours le noeud non traite le moins couteux.

Complexite avec une file de priorite:

`O((V + E) log V)`

avec `V` le nombre de sommets et `E` le nombre d'aretes.

### A*

A* reprend la logique de Dijkstra, mais ajoute une heuristique pour orienter la recherche vers la destination. Dans ce projet, l'heuristique est la distance Haversine a vol d'oiseau entre le noeud courant et la destination. Elle est admissible car elle ne surestime pas la distance routiere.

Complexite theorique dans le pire cas:

`O((V + E) log V)`

En pratique, A* visite souvent moins de noeuds que Dijkstra si l'heuristique guide bien la recherche.

## Matrice de distances

Pour `n` points de livraison plus le depot, on calcule les distances entre toutes les paires de points. La matrice sert ensuite aux heuristiques de tournee.

Complexite:

`O(n^2 * cout_plus_court_chemin)`

C'est couteux si le nombre de points augmente, mais pratique pour comparer les solutions TSP et VRP.

## TSP

### Plus proche voisin

La tournee commence au depot. A chaque etape, on va vers le point non visite le plus proche. C'est rapide mais pas toujours optimal.

Complexite:

`O(n^2)`

### 2-opt

2-opt ameliore une tournee en inversant des segments lorsque cela reduit la distance totale. C'est une methode locale simple et efficace.

Complexite approximative:

`O(k * n^2)`

avec `k` le nombre d'iterations d'amelioration.

## VRP

Pour repartir les livraisons entre plusieurs vehicules, le projet implemente une version simple de Clarke & Wright. L'idee est de commencer avec une route par livraison:

`depot -> client -> depot`

Puis on fusionne les routes lorsque l'economie de distance est interessante et que la capacite du vehicule n'est pas depassee.

L'economie entre deux clients `i` et `j` est:

`s(i,j) = d(depot,i) + d(depot,j) - d(i,j)`

Complexite principale:

`O(n^2 log n)`

car on calcule puis trie toutes les economies.

## Article scientifique lu

J'ai choisi l'article de Clarke et Wright, `Scheduling of Vehicles from a Central Depot to a Number of Delivery Points` publie en 1964.

Idees principales:

- le probleme part d'un depot central et de plusieurs clients a livrer;
- faire une route separee pour chaque client est simple mais inefficace;
- on peut mesurer l'interet de fusionner deux routes avec une economie de distance;
- les meilleures economies sont appliquees en premier, en respectant les contraintes comme la capacite des vehicules.

Lien avec le projet:

- l'algorithme VRP implemente reprend exactement cette idee d'economies;
- la capacite est modelisee par une demande de 1 par client;
- le resultat est rapide a calculer, mais pas garanti optimal.

## Compromis qualite / temps

Dijkstra donne une solution optimale pour le plus court chemin, mais il explore parfois plus largement que necessaire. A* garde l'optimalite si l'heuristique est admissible, tout en reduisant souvent le nombre de noeuds visites.

Pour le TSP, le plus proche voisin est tres rapide mais peut produire une tournee mediocre. 2-opt ajoute du temps de calcul, mais ameliore souvent la distance finale.

Pour le VRP, Clarke & Wright est un bon compromis: il est beaucoup plus rapide qu'une recherche exhaustive, mais il reste heuristique. Il donne une solution credible sans garantir la meilleure solution possible.

## Limites et ameliorations possibles

- Utiliser un vrai graphe OpenStreetMap plus grand.
- Ajouter les sens uniques et les vitesses routieres.
- Comparer avec une solution optimale sur de petites instances.
- Ajouter une metaheuristique comme recuit simule ou algorithme genetique.
- Implementer les Contraction Hierarchies pour accelerer les plus courts chemins.
