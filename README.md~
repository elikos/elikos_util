# elikos_util
elikos_util contient le code qui n'est pas nécessaire à la mission. C'est principalement du code pour les tests.

# Notes sur l'utilisation du VICON au lab du 5ème.  
1. Lien utile pour les paramètres LPE : https://pixhawk.ethz.ch/docs/modules_2local__position__estimator_2params_8c.html#a034f82ab609defe64a477a213cc5ad2b  


## Démarrage VICON
1. S'assurer que les caméras sont branchées à la switch et que la switch est branché au LAN (mur)
1. Connecter le routeur d'élikos au LAN du lab par cable ethernet.  
1. Brancher l'ordi du vicon au routeur d'Élikos.
1. Sur l'ordi du lab, ouvrir la version la plus récente de vicon tracker.
1. Dans la table system, vérifier que les cams sont connectées (vert)
1. Dans la table `calibrate` cliquer sur start dans la section `create camera mask`. S'assurer qu'il n'y a aucun capteurs dans l'arene (ni le magic wand). Ensuite stop.
1. Dans la section `calibrate cameras`, cliquer sur start. Allumer la magic wand et se promener dans l'arène en faisant beaucoup de mouvements (genre des grands huits) avec la magic wand (en pointant les leds le plus possible vers les caméras) jusqu'à ce que tous les triangles disparaissent de l'écran. Attendre que la calibration termine (barre de chargement). Ensuite cliquer sur stop.
1. Déposer la magic wand allumée à l'origine de l'arène.
1. Dans la section `set volume origin`, cliquer sur `start`, s'assurer que la wand est bien détectée. Cliquer sur `start calibration`.
1. Changer la vue pour 3D perspective.
1. S'assurer que les caméras sont bien positionnées selon l'origine définie et les unes par rapport aux autres.

## Utilisation de vicon_bridge
1. Mettre les capteurs du vicon (boules argentées) sur le quad (au moins 3). S'assurer qu'ils ne sont pas mis de façon symétrique.
1. Dans le logiciel, allez dans l'onglet `objects`.
1. Mettre le quad au centre de l'arène.
1. S'assurer qu'aucun objet n'est coché. Vous devriez voir les boules du quad dans l'arene.
1. Sélectionner toutes les boules du quad avec la touche `ctrl` enfoncée.
1. En bas à gauche à côté de l'étiquette `create object`, écrire le nom du quad `elikos_vicon_quad`. Cliquer sur `create`. Le nom devrait s'afficher dans la liste des objets. Il devrait être de couleur jaune ou vert.
1. Mettre l'adresse IP de l'ordinateur dans le launch file `vicon_bridge/launch/vicon.launch`, port 801. Le changement se fait sur la ligne commentée : n'oubliez pas de la décommentée et d'enlever la ligne en dessous.
1. Connecter le quad et un ordi remote au wifi élikos.  
1. Exporter le ROS_MASTER_URI (port 11311) pour l'adresse IP du quad sur l'ordi distant.

## Lancer la localisation sur le quad
1. Lancer `mavros` avec, par exemple, `test_px4.launch`.
1. Lancer le launch file `elikos_vicon_remapping.launch` sur le quad.
1. Lancer le noeud `elikos_origin_init`.
1. Faire l'appel au service `/elikos_origin_init`.
1. Vérifier dans rviz que le frame TF `elikos_fcu` suit `elikos_vision`.

## Avant de partir
1. S'assurer que tout a été rebranché et est propre!