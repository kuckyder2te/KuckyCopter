/*
Autor: Stephan Scholz
Date : 2022:04:30

Erweiterung für LOGGER
Man kann jetzt einen Formatstring übergeben

LOGGER_NOTICE_FMT("/%f,%f,%f,%f,%f,%f,%f/",
                  a.acceleration.x,a.acceleration.y,a.acceleration.z,
                  g.gyro.x,g.gyro.y,g.gyro.z,temp.temperature);
daraus wird
[NOTICE]:virtual void Gyro::update():/-0.593762,0.090980,10.096349,0.029311,0.099657,-0.041568,21.868235/  
https://www.tutorialspoint.com/c_standard_library/c_function_sprintf.htm

Die Gesamtlänge der Nachricht darf 100 Bytes nicht überschreiten

Erweitert : 2022:07:08
Autor: Wilhelm Kuckelsberg
       via "#ifdef _MINITERM" kann nun die Ausgabe zwischen Serial und Serial2 gewechselt werden
       MINITERM ist das interne Terminal von PlatformIO

*/

#include "..\lib\myLogger.h"

char logBuf[100];

void localLogger(Logger::Level level, const char* module, const char* message)
{
  #ifdef _MINITERM    // Ausgabe via USB

    #ifdef LOG_TIMESTAMP
      Serial.print(millis());
      Serial.print(" - ");
    #endif
      Serial.print(F("["));
      Serial.print(Logger::asString(level));
      Serial.print(F("]:"));
      if (strlen(module) > 0)
      {
          Serial.print(module);
          Serial.print(":");
      }
      Serial.println(message);
  #else
    #ifdef LOG_TIMESTAMP
      Serial2.print(millis());
      Serial2.print(" - ");
    #endif
      Serial2.print(F("["));
      Serial2.print(Logger::asString(level));
      Serial2.print(F("]:"));
      if (strlen(module) > 0)
      {
          Serial2.print(module);
          Serial2.print(":");
      }
      Serial2.println(message);
  #endif
}
