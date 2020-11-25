# Door Security Robot
## Problématique
Comment gérer plusieurs portes à distance ?

## Solution
Nous avons prototypé un robot capable de surveiller des portes et d'alerter l'utilisateur en cas d'ouverture

## Lien de la vidéo
<https://www.youtube.com/watch?v=cSmAleeSI_E>
## Equipe 
- Arthur Caron
- Edouard Petitpierre
- Eléonore François

## Encadrant
Sébastien Altounian

## Briques mises en place
- Application mobile (MIT App Inventor) pour envoyer le choix de la porte et recevoir une alerte
- Bluetooth Low Energy pour la communication application/ESP32
- Module ESP32 TTGO
- rosserial pour la communication ESP32/ROS
- Moteur Dynamixel AX-12 pour se diriger vers la bonne porte (contrôlé par ROS)
- Capteur ToF pour la détection
- En cas de détection : l'utilisateur reçoit une alerte, une alarme sonne et une LED rouge s’allume
- Impression 3D : pièce reliant le moteur et le capteur

## Comment exécuter le code
1. Vérifiez la communication avec le moteur : `rosrun dynamixel_driver info_dump.py -p /dev/ttyACM0 2`
2. Executer : `roslaunch my_dynamixel_tutorial controller_manager.launch`
3. Executer dans un autre terminal : `roslaunch my_dynamixel_tutorial controller_spawner.launch`
4. Executer : `roslaunch my_dynamixel_tutorial communication_arduino.launch`

