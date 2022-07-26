/*  File name : putty_out.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-07-05
	Description : Convertieren der Serial.print Ausgaben in ESC Sequenzen
	Putty Einstellungen: 	col 100 row 40
							Font Courier New bold 12 point
						    forbid resizing compleytely
							Disable scrollbar
							local echo beide force off
							Cursor underline blink				
	Refer: https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797
*/

#pragma once

#include <HardwareSerial.h>

class PUTTY_out {

private:
	HardwareSerial &_serial;

public:
	PUTTY_out(HardwareSerial &serial) :	_serial(serial) {}

	virtual void setup() {
	_serial.print("\eSP F");  	// tell to use 7-bit control codes
	_serial.print("\e[?25l"); 	// hide cursor
	_serial.print("\e[?12l");	// disable cursor highlighting
	_serial.print("\e[2J"); 	// clear screen
	} /* end of virtual void setup */

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

	} /*-------------------------- end of print characters ----------------------------*/

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

	} /*-------------------------- end of print double variables ----------------------*/

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

	} /*-------------------------- end of print uint16_t integer variables ------------*/

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
	} /*-------------------------- end of print uint16_t integer variables ------------*/

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
	} /*-------------------------- end of print uint16_t integer variables ------------*/

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
	} /*-------------------------- end of print uint8_t integer variables -------------*/

	void clearLine() {
 	/* Deletes the completely current line. */
		_serial.print("\e[K"); // clear screen ESC[K	clear to end of line
	} /*-------------------------- end of clearLine -----------------------------------*/

	/* Deletes the line according to the number of characters.
	* Beginning by r and c.  */
	void clearPart(uint8_t r, uint8_t c, const char t[]) {
	char space[50];
	char cc[] = " ";
	uint8_t len;
	//uint8_t lastLen;

		len = strlen(t);

		_serial.print("\e[");
		_serial.print(r);
		_serial.print(";");
		_serial.print(c);
		_serial.print("f");

		for(uint8_t i=0; i<=len; i++){
			strcat(space, cc);
		}

	//	lastLen = len;

		_serial.print(space);
	} /*-------------------------- end of clearPart -----------------------------------*/

	void clear() {
	/* Clears the whole screen. */
		_serial.print("\e[2J");
	} /*-------------------------- end of clear ---------------------------------------*/

	void red(){
	/* Print characters red */
		_serial.print("\e[31m");
	} //*-------------------------- end of red ----------------------------------------*/

	void yellow(){
	/* Print characters yellow */
		_serial.print("\e[33m");
	} //-------------------------- end of yellow --------------------------------------*/

	void green(){
	/* Print characters green */
		_serial.print("\e[32m");
	} //-------------------------- end of green ---------------------------------------*/

	void magenta(){
	/* Print characters magenta */
		_serial.print("\e[35m");
	} //-------------------------- end of magenta -------------------------------------*/	
	
	void blue(){
	/* Print characters blue */
		_serial.print("\e[34m");
	} //-------------------------- end of blue ----------------------------------------*/

	void gray(){
	/* Print characters gray */
		_serial.print("\e[37m");
	} //-------------------------- end of gray ----------------------------------------*/
	
	void cyan(){
    /* Print characters cyan */
		_serial.print("\e[36m");
	} //-------------------------- end of cyan ----------------------------------------*/
};

