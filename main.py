#!/usr/bin/env pybricks-micropython
from diagnose        import *
# ueberpruefen ob alle Stecker stecken
# check = False
# while check == False:
#   check = runDiagnose()

#---------------------------------------------#
from missions        import *

# # Programm zur Auswahl der Missionen 
# selection = 0
# missions  = [ "1 Stau, Schaukel", 
#               "2 Kran", 
#               "3 Haeuser", 
#               "4 Bruecke",
#               "5 Innovative Architektur" ]

# #---------------------------------------------#
# def listMissions(index):
#   brick.display.clear()
#   for i in range(0, len(missions)):
#     if (index == i):
#       brick.display.text("[XXX]",   (20, (i+1)*15))
#     else:
#       brick.display.text("[   ]",   (20, (i+1)*15))
#     brick.display.text(missions[i], (70, (i+1)*15))

# #---------------------------------------------#
# def runMission(index):
#   if index == 0:
#     Run01_StauSchaukel()
#   if index == 1:
#     Run02_Kran()
#   if index == 2:
#     Run03_Haus()
#   if index == 3:
#     Run04_Bruecke()
#   if index == 4:
#     Run05_Innovative()

# #---------------------------------------------#
# def handleButton(index):
#   #--- wenn mehrere knöpfe gedrückt werden, passiert nichts
#   if (len(brick.buttons()) != 1):
#     return
#   if (brick.buttons()[0] == Button.DOWN):
#     index = index + 1
#   if (brick.buttons()[0] == Button.UP):
#     index = index - 1
#   if (brick.buttons()[0] == Button.CENTER):
#     runMission(index)  
#   index = index % len(missions)
#   return index 


# #---------------------------------------------#
# while True:
#   #--- Missionen auflisten
#   listMissions(selection)
#   #--- solange kein knopf gedrückdt wird gewartet
#   while not any(brick.buttons()):
#     wait(10)
#   #--- ein Knopf wurde gedrueckt
#   selection = handleButton(selection)
#   #---
#   while any(brick.buttons()):
#     wait(10)



driveSmoothly(300, 100)
#alignBackward()
