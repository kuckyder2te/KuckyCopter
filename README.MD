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
newPID.h eingefügt und pidController.h entfernt
FastPID als Objekt eingefügt. Code wird ausgeführt. Die Eingabe der PID Werte muss noch geändert werden       
_instance in pidNew wird mehr als 3 mal erzeugt, und   nach jedem Reset um 1 oder auch mehr erhöht. gefixt
Achsen- und Motor Klassen hinzugeführt. Nicht ausführbar, nur kompilierbar.  
axisyaw hinzugefügt, motor_state_e in axisMotor hinzugeführt bzw. von state_e umbenannt.
_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::disablePID); geändert in
_yawData->axisData[0]->state = motor_state_e::disablePID;
Zeile 120 axisYaw.
Flycontroller template hinzugefügt
Achsen Klassen hizugefügt. Kein Kompiler Fehler. Muss noch aufgerämt werden.
2022-06-25 Flycontroller.h teileweise implementiert (ohne model Abhänigkeiten)
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
PID Menü wir kurz angezeigt und dann vom Logger überschrieben fix
PID Anzeige auf BT Modul, Logger auf miniterm
LOGGER via Platformio.ini -_MINITERM ein ausschalten
Eingabe via BT funktioniert, kleine Probleme in der Logger-Ausgabe behoben 
Nachkommastellen der Werte via BT einstellen fixed
Ordner EEPROM gelöscht
DallasTemperature und onewire auskommentiert
GUI in putty_out umbenannt
coeffizient_t; in 'DEFINE
_newPID_pri teilweise eingefügt
_newPID als array
displayPIDcoefficientsTemp -> displayPIDcoefficients
_newPID[PID_PRI]->setP ... eingefügt
Tastatureingabe seltsam, mal reagiert das Terminal meistens aber nicht  / fix
Lösung _serial = serial
_MINITERM für Logger eingefügt
Typenbezeichnungen harmonisiert z.B. itemOffset_t;
PID Werte umbenannt
enum in Class verschoben
extEEPROM eingefügt 
PID Werte den Achsen zugewiesen
FlyController überprüft bis ground OK.
PID_adjust, enum in die Klasse verschoben
Dictionary als Übersetzungstabelle implementiert
axisBase enum in Klasse verschoben
def.h gelöscht
PID_new intFastPID in Constructor verschoben
getExecutionTime wird nicht ausgeführt
axisName_t statt #define
PID_adjust setModel eingefügt
radio.h Zeilen 107-112 
189-207 auskommentiert alles OK
CleanUp

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

In der Klasse radio.h wird die Varialble payload_new nicht in Zeile 165
LOGGER_NOTICE_FMT("Received %d bytes of pipe %d Payload new %f", bytes, pipe, payload_new);
angezeigt bzw. hochgezählt.
Ich vermute, es liegt an der 
"uint8_t address[][6] = {"1Node", "2Node"};
Ich vermute, ich habe es nicht richtig deklariert bzw. initialisiert
Lösung; Im Sender den Varialbennamen auf payload_geändert????

# Hinweise
Will man die Achsen via Logger debuggen, muss in axisBase.h und motor.h am Dateiende das #undef auskommentziert werden "// #undef _DEBUG_"


                                                       
