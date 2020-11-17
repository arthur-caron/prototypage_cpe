# Door-watcher
## Problématique
Prototyper un robot capable de surveiller des portes et de détecter une ouverture

## Lien de la vidéo

## Equipe 
- Arthur Caron
- Edouard Petitpierre
- Eléonore François

## Encadrant
Sébastien Altounian

## Briques mises en place
- Application mobile pour le choix de la porte 
- Communication BLE Application / ESP 32 pour envoyer le choix
- Moteur Dynamixel pour se tourner vers la bonne porte
- Détection seulement si le moteur est à l’arrêt
- En cas de détection : l’alarme sonne et la LED rouge s’allume
- Impression 3D : pièce reliant le moteur et le capteur

## Comment exécuter le code
- Vérifiez la communication avec le moteur : rosrun dynamixel_driver info_dump.py -p /dev/ttyACM0 2
- Executer : roslaunch my_dynamixel_tutorial controller_manager.launch
- Executer dans un autre terminal : roslaunch my_dynamixel_tutorial controller_spawner.launch
- Excetuer : roslaunch my_dynamixel_tutorial communication_arduino.launch

