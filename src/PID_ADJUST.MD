#   Einstellen der PID Werte
    Vorbeeitung
    Platformio.ini  MINITERM und PID_ADJUST aktivieren
    BT COM öffnen
    Programm starten, Hauptmenü öffnet sich

##  Achse auswählen
    "X" wählt die primäre Achse aus und setzt mit setItemAxis() eine "1"
    Anschließend
    "P" wählt den ersten Parameter und setzt mit setItemOffset() eine "10"

    "+" oder "-" wählt coefficient_Up() 
                                        select(getPidType(true)) addiert
                                        setItemAxis() + setItemOffset()
                                        und git den PIDtype "11" zurück
                                        In diesem Fall den P-Wert der X-Achse

                                        Außdem wird entschieden, ob sich der Wert
                                        erhöht oder erkleinert. Zudem wird die Genauigkeit
                                        festgelegt.
        
        In der Funktion void select(uint8_t type) wird dem PIDType der Wert übergeben.
            pri_kP_value