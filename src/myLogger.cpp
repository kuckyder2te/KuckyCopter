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
*/

#include "..\lib\myLogger.h"

char logBuf[100];

void localLogger(Logger::Level level, const char* module, const char* message)
{
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
}
