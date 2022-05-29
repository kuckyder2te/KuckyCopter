#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include <HardwareSerial.h>
#include "myLogger.h"

class Gui : public Task::Base {

private:
	HardwareSerial &_serial;

public:
    Gui(const String& name)
    : Task::Base(name),_serial(Serial) {
    }

    virtual ~Gui() {}

    virtual void begin() override {
		LOGGER_VERBOSE("Enter....");
        _serial.print("\eSP F");  	// tell to use 7-bit control codes
        _serial.print("\e[?25l"); 	// hide cursor
        _serial.print("\e[?12l");	// disable cursor highlighting
        _serial.print("\e[2J"); 	// clear screen 
		LOGGER_VERBOSE("....leave");

		delay(1000);
    }

    // optional (you can remove this method)
    // virtual void enter() override {
    // }

    Gui* port(const HardwareSerial &p) {
        _serial = p;
        return this;
    }
    virtual void update() override {
		LOGGER_VERBOSE("Enter...");
		LOGGER_VERBOSE("...Leave");
	//	delay(10000);
    }

    void print(uint8_t r, uint8_t c, const char t[]) {

    /* Prints a array of char at position r and c.
    * and calculates the number of characters. */
		_serial.print("\e[1m");
		_serial.print("\e[");  	// escape symbol
		_serial.print(r); 		// column
		_serial.print(";");
		_serial.print(c);		// row
		_serial.print("f");
		_serial.print(t);		// text

	} //-------------------------- end of print characters ----------------------------------------

	void print(uint8_t r, uint8_t c, uint8_t dot, double d) {
	/* Prints a double type at position r and c. */

		_serial.print("\e[1m");
		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");
		_serial.print(d, dot);
		_serial.print("\e[0m");

	} //-------------------------- end of print double variables ----------------------------------

	void print(uint8_t r, uint8_t c, int16_t i16) {
	/* Prints a integer type at position r and c. */

		_serial.print("\e[1m");
		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");
		_serial.print(i16);
		_serial.print("\e[0m");

	} //-------------------------- end of print uint16_t integer variables ------------------------

	void print(uint8_t r, uint8_t c, uint32_t ui32) {
	/* Prints a integer type at position r and c. */

		_serial.print("\e[1m");
		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");
		_serial.print(ui32);
		_serial.print("\e[0m");

	} //-------------------------- end of print uint16_t integer variables ------------------------

	void print(uint8_t r, uint8_t c, uint16_t ui16) {
	/* Prints a integer type at position r and c. */

		_serial.print("\e[1m");
		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");
		_serial.print(ui16);
		_serial.print("\e[0m");

	} //-------------------------- end of print uint16_t integer variables ------------------------

	void print(uint8_t r, uint8_t c, uint8_t ui8) {
	/* Prints a integer type at position r and c. */

		_serial.print("\e[1m");
		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");
		_serial.print(ui8);
		_serial.print("\e[0m");

	} //-------------------------- end of print uint8_t integer variables -------------------------

	void clearLine() {
 	/* Deletes the completely current line. */

		_serial.print("\e[K"); // clear screen ESC[K	clear to end of line

	} //-------------------------- end of clearLine -----------------------------------------------

 	/* Deletes the line according to the number of characters.
  	* Beginning by r and c.  */
	void clearPart(uint8_t r, uint8_t c, const char t[]) {
	char space[50];
	char cc[] = " ";

		uint8_t len = strlen(t);

		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");

		for(uint8_t i=0; i<=len; i++){
			strcat(space, cc);
		}
		_serial.print(space);

	} //-------------------------- end of clearPart -----------------------------------------------

	void clear() {
	/* Clears the whole screen. */
	LOGGER_WARNING("Enter....");
		_serial.print("\e[2J");
	LOGGER_WARNING("....leave");

	} //-------------------------- end of clear ---------------------------------------------------
	void red(){
	/* Print characters red */
		_serial.print("\e[31m");

	} //-------------------------- end of red -----------------------------------------------------

	void yellow(){
	/* Print characters yellow */
		_serial.print("\e[33m");

	} //-------------------------- end of yellow --------------------------------------------------

	void blue(){
	/* Print characters blue */
		_serial.print("\e[34m");

	} //-------------------------- end of blue ----------------------------------------------------

	void gray(){
	/* Print characters gray */
		_serial.print("\e[37m");

	} //-------------------------- end of gray ----------------------------------------------------
    // optional (you can remove this method)
    // virtual void exit() override {
    // }

    // optional (you can remove this method)
    // virtual void idle() override {
    // }

    // optional (you can remove this method)
    // virtual void reset() override {
    // }


};