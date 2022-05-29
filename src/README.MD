weger Adafruit libs I²c und SPI in der main
alle Wire und SPI auskommentiert

Der NRF24 scheint keinen CE und CSN Pin zu benötigen

Der Dallas SEnsonr funktioniert nicht

Battery Klasse noch nicht Hardware getestet

BMP280 Werte werden wohl nicht korrekt ausgelesen. Adresse ??

MadgwickQuaternionUpdate eingefügt, noch nicht getestet

Calibration Klasse eigefügt. Muss alle 100ms ausgeführt werden
    Noch keine weiterleitung an GUI Klasse
    Befehle werden icht an GUI weiter geleitet. Wahscheilich liegstam PUTTY Code

Stephans Logger Warning für die Calibration Klasse verwendet    

!!! Wichtig:
    Solange die Calibration Klasse getest wird, muss nach dem Haupt-Logo die "0" einmal gedrückt werden

pidController.cpp    