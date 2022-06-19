weger Adafruit libs I²c und SPI in der main
alle Wire und SPI auskommentiert

Der NRF24 scheint keinen CE und CSN Pin zu benötigen

Der Dallas Sensonr funktioniert nicht

Battery Klasse noch nicht Hardware getestet

BMP280 Werte werden wohl nicht korrekt ausgelesen. Adresse ??

MadgwickQuaternionUpdate eingefügt, noch nicht getestet. Entfernt

Calibration Klasse eingefügt. Muss alle 100ms ausgeführt werden
    Noch keine weiterleitung an GUI Klasse
    Befehle werden nicht an GUI weiter geleitet. Wahrscheilich liegt es am PUTTY Code

Stephans LOGGER_WARNING_FMT für die Calibration Klasse verwendet    

!!! Wichtig:
    Solange die Calibration Klasse getestet und ausgeführt wird,
    muss nach dem Haupt-Logo die "0" einmal gedrückt werden um den NRF zu aktivieren 

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
    ## Fix 2    2022-06-03  Code wird wieder mit allen Instanzen aufgeführt.
    hängt aber hier :case pidTyp_t::pri_P: 
			                                    _x_kP += _factor;
			                                    LOGGER_WARNING_FMT("X kP = %f", _x_kP);
			                                    _pid_pri->setP(_x_kP);
                                                Y kP = 20505159319395510517760.000000 gefixt
    ## Fix 3    PID values mit 0 initialisiert    
                2022-06-03  Verhindern, dass PID Werte nicht <0 werden. gefixt:
    ## Fix 4    2022-06-04  pidNew.h eingefügt und pidController.h entfernt
    FastPID als Objekt eingefügt. Code wird ausgeführt. Die Eingabe der PID Werte muss noch geändert werden       
    ## Fix 5    2022-06-xx  _instance in pidNew wird mehr als 3 mal erzeugt, und   nach jedem Reset um 1 oder auch mehr erhöht
    ## Fix 6    2022-06-06  EEPROM Klasse hinzu gefügt, aber nicht nicht ausführbar undefined reference to `EEPROMClass::EEPROMClass   
    ## Fix 7    2022-06-06  Achsen- und Motor Klassen hinzugeführt. Nicht ausführbar, nur kompilierbar.  
    ## Fix 8    2022-06-08  Achsen und Motor-Klassen wieder entfernt     
    ## Fix 9    2022-06-13 Calibration und PID entfernt      
    ## Fix 10   2022- 06-13  baseAxis und newPID hinzugefügt Instance Problem besteht weiter, _newPID->step(*_sp, *_fb)   ->step erscheint nicht in der Auswahlliste. Telefon mit Stephan. Klassen umbenannt axisyaw entfernt. static entfernt um die App komplierbar zu machen. Adafruit_PWMServoDriver.h eingefügt.
    ## Fix 11   2022-06-14 Logger in motor hinzugefügt. Motor Klasse geändert (axisMotor Zeile 58) 
    ## Fix 12   Instance  Problem geklärt
    ## Fix 13   MotorAxis released
    ## Fix 14   2022-06-17 axisyaw hinzugefügt, motor_state_e in axisMotor hinzugeführt bzw. von state_e umbenannt.
    _motorAxis[axis_t::Primary]->setState(modules::AxisMotor::disablePID); geändert in
    _yawData->axisData[0]->state = motor_state_e::disablePID;
    Zeile 120 axisYaw.
    ## Fix 15    2022-06-19 Flycontroller template hinzugefügt