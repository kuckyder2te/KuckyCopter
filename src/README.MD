# Allgemeine Informationen
Adafruit libs I²c und SPI in der main, andere Wire und SPI auskommentiert.
Der Dallas Sensonr funktioniert nicht
Battery Klasse noch nicht Hardware getestet
BMP280 Werte werden wohl nicht korrekt ausgelesen. Adresse ??
MadgwickQuaternionUpdate eingefügt, noch nicht getestet. Entfernt
PID_adjust Klasse eingefügt. Muss alle 100ms ausgeführt werden

The parameter P domain is [0.00390625 to 255] inclusive.
Last PID parameter
P		I		D
Pri		0,14	0,18	0,102
Sec		0,14	0,18	0,102
Yaw		0,01	0		0

PID values mit 0 initialisiert    
Verhindern, dass PID Werte nicht <0 werden. gefixt:
Flycontroller template hinzugefügt
Achsen Klassen hinzugefügt. Kein Kompiler Fehler. Muss noch aufgerämt werden.
2022-06-25 Flycontroller fertig. Stichworte
                flyState_e _flyState;
                AxisYaw *_axisYaw;
                yawData_t *_yawData;
                Radio *_radio;
                Sonic *_sonicData;
PWM_RP2040.h implementiert. Fehler bei motor.h Zeile 98  und battery.h Zeile 49   gefixt
Platformio.ini upload_port = picotool. Keine Änderung zu erkennen   
Programm läuft durch bis Flycontroller.h Zeile 82. Stats werden nicht richtig übergeben  gefixt 
Im Flycontroller eine Testaufruf auf AxisYaw hinzugefügt und funktioniert, States werden aufgerufen auf, motor gelöst
Define MotorPins in main.
                                            Sensors 123
                                            sonic Temperature
PID_adjust und GUI erolgreich implementiert. 

Eingabe via BT funktioniert, kleine Probleme in der Logger-Ausgabe behoben 
Nachkommastellen der Werte via BT einstellen fixed
Ordner EEPROM gelöscht
DallasTemperature und onewire auskommentiert
GUI in putty_out umbenannt
coeffizient_t; in 'DEFINE
_newPID_pri teilweise eingefügt
_newPID als array

LOGGER via Platformio.ini 
_MINITERM enable via USB disable via BT
PID Werte den Achsen zugewiesen
FlyController überprüft bis ground OK.
Dictionary als Übersetzungstabelle implementiert

Radio class mit "GetStarted" gefüllt. Das Gegenstück ist auf dem MEGA2560

[Change 101]	Flashen geht wieder, abewr mur durch verschieben in das Pico Laufwerk
				Board auf rpipico geändert. UART für Serial2 ist nicht mehr nötig
				PID justierung kann in der PlatformIO ein/aus-geschaltet werden.
				In radio.h payload von den RC implemetiert, aber noch keine Verbindung
				Kosmetik
[Change 102]	Radio funktioniert wegen Änderung der Pipe "Kennung"
				EEPROM eingefügt	
[Change 103]	payload Werte auf float gesetzt. Transmission OK, aber Werte nicht OK.  Fix
				Sollten rcSw1 etc. besser als bool deklariert werden?	
				Überflüssige Kommentare entfernt	
[Change 104]	Nur Kosmetik	
[Change 105]	Transmission und Werte OK	
[Change 106]	interface to rcInterface fix	
[Change 107]	RC Switch Typ in bool geändert fix	
[Change 108]	EEPRPM vorbereitet. RC OK	
[Change 109]	Radio started im Empfangsmodus automatisch, ohne Tastendruck.

# Fragen
    // Model wiring begin
	// remote control channels to MotorAxis
	model.yawData.axisData[axis_t::Primary].rcX = &model.interface.payload.rcDirection[axis_t::Primary];
	model.yawData.axisData[axis_t::Primary].rcY = &model.interface.payload.rcDirection[axis_t::Secondary];
	model.yawData.axisData[axis_t::Secondary].rcX = &model.interface.payload.rcDirection[axis_t::Primary];
	model.yawData.axisData[axis_t::Secondary].rcY = &model.interface.payload.rcDirection[axis_t::Secondary];
	model.yawData.rotationSpeed = &model.interface.payload.rcDirection[axis_t::YawAxis];

	model.yawData.axisData[axis_t::Primary].feedback = &model.gyroData.imuOutput[axis_t::Primary];
	model.yawData.axisData[axis_t::Secondary].feedback = &model.gyroData.imuOutput[axis_t::Secondary];
	model.yawData.feedback = &model.gyroData.imuOutput[axis_t::YawAxis];
	model.yawData.horz_Position = &model.gyroData.horz_Position;      
	muss das wieder gemacht werden?  

## Hinweise
Will man die Achsen via Logger debuggen, muss in axisBase.h und motor.h am Dateiende das #undef auskommentiert werden "// #undef _DEBUG_"
Bei ARM Prozessoren hat double eine länge von 8byte und float von 4byte

## Transmission
_radio.openWritingPipe(pipes[0]);  <-
_radio.openReadingPipe(1, pipes[1]);  <-
radio.stopListening(); // put radio in TX mode

## Receiver
_radio.openWritingPipe(pipes[1]);  <-
_radio.openReadingPipe(1, pipes[0]); <-
_radio->startListening(); // put radio in RX mode

## Nächste Projekt-Schritte
	1.	Radio Daten verfizieren fix
	2.	EEPROM aktivieren
	3.	Radio nur auf Receiver schalten
	4.	PWM Signal auf Motoren übertragen
	5.	LOGGER überprüfen
	6.	Sensordaten überprüfen und ggf. Filter einfügen
	7.	NRF24 printDetails überprüfen
	8.  PID Werte aus PID-adjust in Model einlesen


##
 	Achsen-Bezeichnung zentral ablegen ??
	Interface in rcInterface umbenennen fix 
	Payload rcSwix	-> bool   fix                                                 
