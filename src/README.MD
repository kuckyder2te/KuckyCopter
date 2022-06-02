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
    Solange die Calibration Klasse getestet und ausgeführt wird,
    muss nach dem Haupt-Logo die "0" einmal gedrückt werden   

    The parameter P domain is [0.00390625 to 255] inclusive.
    Last PID parameter
    	P		I		D
    Pri		0,14	0,18	0,102
    Sec		0,14	0,18	0,102
    Yaw		0,01	0		0

    Als nächstes:
    calibration Zeile 355  pid_pri.setP(((_model->pidData[axis_t::Pr
                        -model nicht gefunden

    # Fixes
    ## Fix 1    2022-06-01  Komplierfähiger Code, nach dem flash keine COM Ausgabe   