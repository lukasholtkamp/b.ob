"""
Eingabemodul fuer X-Box und Hama Pad
V1.0 16.04.2014
Autor: Michael Lartz
Kontakt: bmxhome@gmx.de
Projekt: www.forum-raspberrypi.de/Thread-python-joypad-abfrage-modul
"""

import pygame
from  time import sleep

############################################################
bnames = ["",
          "A",
          "B",
          "X",
          "Y",
          "L1",
          "R1",
          "L2",
          "R2",
          "L3",
          "R3",
          "SELECT",
          "START",
          "XBOX",
          "UP",
          "DOWN",
          "LEFT",
          "RIGHT"]
############################################################

## Joypad setup
pygame.init()
pygame.joystick.init()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    name = joystick.get_name()
    eingesteckt = True

    if name == "GreenAsia Inc.      USB  Joystick  ":
        bjoy = [ 0, 2, 1, 3, 0, 4, 5, 6, 7,10,11, 8, 9,99,96,95,97,98]
        ajoy = [1,0,2,3]
        bekannt = True
        
    elif name == "Xbox 360 Wireless Receiver":
        bjoy = [ 0, 0, 1, 2, 3, 4, 5,98,97, 9,10, 6, 7, 8,13,14,11,12]
        ajoy = [1,0,4,3,2,5]
        bekannt = True

##    elif name == "Platz fuer weitere Joypads":
##        bjoy = []
##        ajoy = []
##        bekannt = True

    else:
        bekannt = False

except Exception:
    eingesteckt = False

############################################################

def joy_a():
    """
    Gibt Zustand der Analog-Achsen wieder
    Werte 1 und -1 werden fuer bessere Nullpunktbestimmung abgeschnitten
    joy_a()[0] = Linker Stick vertikal    (-10 = UP  , +10 = DOWN )
    joy_a()[1] = Linker Stick horizontal  (-10 = LEFT, +10 = RIGHT)
    joy_a()[2] = Rechter Stick vertikal   (-10 = UP  , +10 = DOWN )
    joy_a()[3] = Rechter Stick horizontal (-10 = LEFT, +10 = RIGHT)
    joy_a()[4] = L2 Regler (falls vorhanden) (0 - 10)
    joy_a()[5] = R2 Regler (falls vorhanden) (0 - 10)
    """

    if not eingesteckt: return "Kein Joypad angeschlossen"
    if not bekannt: return name + " nicht bekannt"

    ausgabe = []
    pygame.event.get()
    for a in range(len(ajoy)):
        b = int((joystick.get_axis(ajoy[a]) / .91) * 10 )
        if a > 3: b = (b + 10) / 2
        if b in (-1,1): b = 0
        ausgabe += [b]
    return ausgabe

############################################################

def joy(taste,typ):
    """
    Gibt Zustand der Buttons wieder
    0    , typ  = alle Tasten abfragen
    1-17 , typ  = einzelne Taste abfragen
    taste,  0   = sofortige Abfrage
    taste,  1   = auf Tastendruck warten
    """

    if not eingesteckt: return "Kein Joypad angeschlossen"
    if not bekannt: return name + " nicht bekannt"
    
    if taste in range(0,len(bjoy)) and typ in (0,1):

####################

        if name == "Xbox 360 Wireless Receiver":
            while 1:
                ausgabe = False
                pygame.event.get()
                if taste > 0 and bjoy[taste] != 99:
                    if   bjoy[taste] == 98: ausgabe = (joystick.get_axis(2) > 0) * 1
                    elif bjoy[taste] == 97: ausgabe = (joystick.get_axis(5) > 0) * 1
                    else: ausgabe = (joystick.get_button(bjoy[taste]) is 1 ) * 1
                elif taste == 0:
                    ausgabe = [False,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    for a in range(1, len(bjoy) ):
                        if bjoy[a] != 99:
                            if   bjoy[a] == 98: ausgabe[a] = (joystick.get_axis(2) > 0) * 1
                            elif bjoy[a] == 97: ausgabe[a] = (joystick.get_axis(5) > 0) * 1
                            else: ausgabe[a] = joystick.get_button(bjoy[a])
                        if ausgabe[a] == 1: ausgabe[0] = bnames[a]
                if typ   == 0                        : break
                if taste >  0 and ausgabe    != False: break
                if taste == 0 and ausgabe[0] != False: break
                sleep(.1)
            return ausgabe
        
####################

        if name == "GreenAsia Inc.      USB  Joystick  ":
            while 1:
                ausgabe = False
                pygame.event.get()
                if taste > 0 and bjoy[taste] != 99:
                    if   bjoy[taste] == 98: ausgabe = (joystick.get_hat(0)[0] is 1) * 1
                    elif bjoy[taste] == 97: ausgabe = (joystick.get_hat(0)[0] is -1) * 1
                    elif bjoy[taste] == 96: ausgabe = (joystick.get_hat(0)[1] is 1) * 1
                    elif bjoy[taste] == 95: ausgabe = (joystick.get_hat(0)[1] is -1) * 1
                    else: ausgabe = ( joystick.get_button(bjoy[taste]) is 1 ) * 1
                elif taste == 0:
                    ausgabe = [False,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    for a in range(1, len(bjoy) ):
                        if bjoy[a] != 99:
                            if   bjoy[a] == 98: ausgabe[a] = (joystick.get_hat(0)[0] is 1) * 1
                            elif bjoy[a] == 97: ausgabe[a] = (joystick.get_hat(0)[0] is -1) * 1
                            elif bjoy[a] == 96: ausgabe[a] = (joystick.get_hat(0)[1] is 1) * 1
                            elif bjoy[a] == 95: ausgabe[a] = (joystick.get_hat(0)[1] is -1) * 1
                            else: ausgabe[a] = joystick.get_button(bjoy[a]) * 1
                        if ausgabe[a] == 1: ausgabe[0] = bnames[a]
                if typ   == 0                        : break
                if taste >  0 and ausgabe    != False: break
                if taste == 0 and ausgabe[0] != False: break
                sleep(.1)
            return ausgabe

####################

##        if name == "Platz fuer weitere Joypads"

####################
        
    return "Wert ("+str(taste)+","+str(typ)+") unzulaessig fuer "+name

############################################################

if __name__ == "__main__":
    while 1:
        try:
            print (joy(0,0), joy_a())
        except KeyboardInterrupt:
            break
        sleep(.1)
    pygame.quit()
