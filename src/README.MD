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
[Change 110]	Radio Sender freigestellt OK
[Change 110a]	Nur Kleinigkeiten, Radio OK
[Change 111]	Logger wieder OK, Radio OK
[Change 112]	LOGGER wieder OK, muss aber noch ausgetestet werden.
[Change 113]	Termin Stephan, 2022.09.20 - Die Achsen axisBase, axisYaw und axisMotor zusammengefasst
				EEPROM implementiert, aber nicht kompilierbar. newPID Zeile 77-79.

New branch Radio ping pong
[Change 114a]	radio.h Variablen RX... und TX... angepasst	. Kompilieren OK
[Change 115]	Radio sendet vom Testprogramm und sendet und empfängt im Flycontroller fix
[Change 115a]	rcinterface_t umbenannt nach RC_interface_t

New branch EEPROM testen
[Change 116]	setItemOffset umbenannt nach setItemCoefficient
				itemOffset_t umbenannt nach itemCoefficient_t
[Change 117]	2022.09.28 Online mit Stephan. EEPROM jetzt in der main includiert fix
				PID Werte kommen als inf bei loadParameters an	
				2022.09.30 Online EEPROM weiter konfiguriert, aber noch nicht richtig.	
[Change 117a]	Nichst besonderes	
[Change 118]	MS5611 und MPU9250 fix (EEPROM noch nicht komplett)	
				Adafruit libs wieder deaktiviert	
[Change 118a]	Sonic ist OK, sendet aber in dieser App keine konstanten Werte, im Test schon.
[Change 118b]	Sonic auf max. 200cm unt timeout auf 11655 microSec. gesetzt.	Fix
				LIB Versions Nummern eingefügt
[Change 119]	Dalles Sensor entfernt und DHT22 implementiert
[Change 119a]	DHT22 fix
[Change 120]	Sonic inkl. Sealevel und Temperatur Kompensation
[Change 121]				save- loadparameter und NewPID::init wird nur 3 * aufgerufen. Werte Siehe newPID_Test_01
new bracnch
[Change 122]	model in PID_adjust includiert, OK
				axisName_t in model.h verschoben und pidData_TEST_t als Test eingefügt OK
[Change 123]	pidData_TEST_t ins model übernommen	OK	
[Change 123a]	Anzeige der PID Parameter wird nun aus dem Model gefüttert
[Change 123b]	axisName_t nach def.h verschoben OK
[Change 123c]   axis_t in axisYaw.h umbenannt in axisName_t
[Change 124d]	pidParameter in pidData "übertragen" ohne PID_adjust OK
[Change 124e]	"" jetzt mit PID_adjust OK
[Change 124f]	Es werden für jede Achse PID Werte in saveParameter zugeteilt. Werte Siehe newPID_Test_02
[Change 124g]	Daten werden saveParameter komplett ausgegeben, bei loadParameter fehlt der 1, Satz Daten.  Werte Siehe newPID_Test_03
[Change 124f]	loadParameter gibt nur die Test-Daten aus. Kein Unterschied zu gespeicherten Daten. Testdaten zurückgesetzt.
[Change 124h]	Menüsteuerung angepasst. PID Werte im Model?? Im model.h alle inkluds disabled
[Change 124i]	Menüsteuerung X, Y, Z Achse OK. Exec. frequency stimmen diePositionen nicht.
[Change 125]	OOP Fehler korrigiert und auskommentierte Teile gelöscht
[Change 126]	EEPROM OK
[Change 127]	Local Logger eingerichtet
[Change 128]	Menü funtioniert wieder
[Change 129]	EEPROM bis auf Z P OK
[Change 130]	EEPROM doch seltsam
[Change 131]	PID Menü auch mit (+) und (-) OK
[Change 132]	PID Adjust teilweise geklärt
[Change 133]	EEprom OK ?, aber YAW axis P hat Anfangs immer 55.600
[Change 134]	EEPROM Liest und schreibt aber nicht immer reproduzierbar
[Change 135]	Kosmetic
				

### **WICHTIG**
In der Klasse Sensor muss die Fehlermeldung "FATAL_ERROR....." wieder einkommentiert werden

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
	Macht es Sinn, die PID Werte i, Model zu speichern?

## Hinweise
Bei ARM Prozessoren hat double eine Länge von 8byte und float von 4byte
Deim debuggen/compilieren muss PID_adjust in Platformio.ini enable sein

## Debug flags
### Logger
Wenn z.B. -D_DEBUG_=FATAL gesetzt wird, werden alle vorherrigen unterdrückt.
In der zu bebuggenden Datei wird vor myLogger.h ein #define LOCAL_DEBUG gesetzt
In myLogger.h wird "#if defined LOCAL_DEBUG || defined GLOBAL_DEBUG" eingefügt und in Platformio.ini mit 
"-DGLOBAL_DEBUG" ausgewählt ob alle Dateien, oder nur spezielle debugt werden.
Man kann 

### COM Ausgabe
	1. -D_MINITERM enable = Ausgabe auf PICO COM Port (Serial)
	2. -D_MINITERM disable = Ausgabe auf PICO COM BT (Serial2)
	3. -D_PID_ADJUST + -D_MINITERM enable = PID Menü auf Serial2 (BT) Logger auf Serial1

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
	2.	EEPROM aktivieren fix
	3.	Radio nur auf Receiver schalten fix
	4.	PWM Signal auf Motoren übertragen
	5.	LOGGER überprüfen fix
	6.	Sensordaten überprüfen und ggf. Filter einfügen
	7.	NRF24 printDetails überprüfen
	8.  PID Werte aus PID-adjust in Model einlesen. Nicht mehr nötig fix

##
 	Achsen-Bezeichnung zentral ablegen fix
	Interface in rcInterface umbenennen fix 
	Payload rcSwix	-> bool   fix   

	PID Menü. Wenn man vor der Achsenwahl "P" drückt, erscheint dieAusgabe zu hoch                                              
