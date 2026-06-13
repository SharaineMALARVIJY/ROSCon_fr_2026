# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Taynaud
# juillet 2023


from vehicle import Driver
from controller import Lidar

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())

sensorTimeStep = 80
if sensorTimeStep < basicTimeStep:
    sensorTimeStep = basicTimeStep
else:
    sensorTimeStep = ((sensorTimeStep + basicTimeStep - 1) // basicTimeStep) * basicTimeStep

#Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
# lidar.enablePointCloud()

#clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# vitesse en km/h
speed = 0
maxSpeed = 28 #km/h

# angle de la direction
angle = 0
maxangle = 0.28

# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)
# mode manuel et mode auto desactive
modeManuel = False
modeAuto = True
print("cliquer sur la vue 3D pour commencer")
print("m pour mode manuel, a pour mode auto, n pour stop, l pour affichage données lidar")
print("en mode manuel utiliser les flèches pour accélérer, freiner et diriger")

while driver.step() != -1:

    speed = driver.getTargetCruisingSpeed()

    while True:
        #acquisition des donnees du lidar
        donnees_lidar = lidar.getRangeImage()
        
        # recuperation de la touche clavier
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break
        if currentKey == ord('m') or currentKey == ord('M'):
            if not modeManuel:
                modeManuel = True
                modeAuto = False
                print("------------Mode Manuel Activé---------------")
        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeManuel or modeAuto :
                modeManuel = False
                modeAuto = False
                print("--------Modes Manuel et Auto Désactivé-------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto : 
                modeAuto = True
                modeManuel = False
                print("------------Mode Auto Activé-----------------")
        elif currentKey == ord('l') or currentKey == ord('L'):
                print("-----donnees du lidar en metres sens horaire au pas de 1°-----")
                for i in range(len(donnees_lidar)) :
                    print(f"{donnees_lidar[i]:.3f}   ", end='')
                    if (i+1)%10 == 0 :        
                       print()
                print()
      
        # Controle en mode manuel
        if modeManuel:
            if currentKey == keyboard.UP:
                speed += 0.2
            elif currentKey == keyboard.DOWN:
                speed -= 0.2
            elif currentKey == keyboard.LEFT:
                angle -= 0.04
            elif currentKey == keyboard.RIGHT:
                angle += 0.04

    if not modeManuel and not modeAuto:
        speed = 0
        angle = 0
        
    if modeAuto:
        speed = 5.4  # km/h
        # angle = donnees_lidar[240]-donnees_lidar[120]
        left_indices = range(110, 131)
        right_indices = range(230, 251)

        left_values = [
            donnees_lidar[i]
            for i in left_indices
            if donnees_lidar[i] != float('inf')
        ]

        right_values = [
            donnees_lidar[i]
            for i in right_indices
            if donnees_lidar[i] != float('inf')
        ]

        if len(left_values) > 0 and len(right_values) > 0:
            left_dist = sum(left_values) / len(left_values)
            right_dist = sum(right_values) / len(right_values)

            error = right_dist - left_dist

            Kp = 0.35
            angle = Kp * error

        else:
            # Si le lidar donne des mesures invalides, on évite un coup de volant brutal
            angle = 0.8 * angle

    # clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > maxangle:
        angle = maxangle
    elif angle < -maxangle:
        angle = -maxangle

    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)

