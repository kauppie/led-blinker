/*
* ledikello.c
*
* Created: 30/10/2019 15.41.52
* Author : Elias, Toivo
*/

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <time.h>

#include <math.h>

// Row and column length.
#define ROWS 16

// Timing constants.
#define ROW_DELAY_US 1000
#define LED_DUTY_CYCLE 100
// With 16-bit timer and 1/64 prescaler on F_CPU.
#define USECS_PER_TIMER_OF 524288
#define DEBOUNCE_TIME_MS 20

// Rendering constants.
#define DIGIT_ROWS 5
#define DIGIT_COLUMNS 3
#define PI 3.14159265358979323846

// Time constants.
#define TIME_ADDRESS 0x100
#define TIME_WRITE_PERIOD_MS 30000

// Macro functions.
#define MIN(X, Y) (X < Y ? X : Y)
#define MAX(X, Y) (X < Y ? Y : X)

// Flags.
#define BUTTON_STATE 0x01
#define SLEEP_STATE 0x02
#define USE_DIGITAL_DISPLAY_STATE 0x04

// Global interrupt controlled volatiles.
volatile uint_fast16_t usecs;
volatile uint_fast32_t msecs;
volatile uint8_t stateFlags;
volatile uint8_t prevStateFlags;
volatile uint8_t buttonPressTime;
volatile uint_fast32_t lastTimeWrite;

// Digits used in digital clock scaled to 5x3 matrix as bits.
const uint16_t digits[10] = {0x7B6F, 0x2492, 0x73E7, 0x73CF, 0x5BC9, 0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF};
// Rendering target and display source.
uint16_t screen[ROWS];

// Function declarations.
void setup();
void powerDown();
void writeTimeToEEPROM();
void readTimeFromEEPROM();
static inline void debounce();
inline void setState(uint8_t);
inline void clearState(uint8_t);
inline void toggleState(uint8_t);
void updateButtonState();
void clearAll();
void clearForDraw();
void allOn();
void setRow(uint8_t);
void setColumn(uint16_t);
void renderDigitalClockOnScreen();
void renderAnalogClockOnScreen();
void clearScreen();


int main()
{
	setup();
	
	while (1) {
		
		if (stateFlags & SLEEP_STATE)
			powerDown();
		
		clearScreen();
		
		if (stateFlags & USE_DIGITAL_DISPLAY_STATE)
			renderDigitalClockOnScreen();
		else
			renderAnalogClockOnScreen();
		
		// Display screen with preset brightness.
		for (uint8_t i = 0; i < ROWS; ++i) {
			clearForDraw();
			_delay_us(ROW_DELAY_US / 100 * (100 - LED_DUTY_CYCLE));
			setRow(i);
			setColumn(screen[i]);
			_delay_us(ROW_DELAY_US / 100 * LED_DUTY_CYCLE);
		}
		// Prevent the last row blasting at full brightness.
		clearForDraw();
	}
}

// Timed interrupt which increments the clock and checks for
// button press.
ISR(TIMER1_OVF_vect) {
	usecs += 531;
	msecs += 529 + usecs / 1000;
	usecs %= 1000;
	// Reset after 24 hours.
	msecs %= 86400000;
	
	if (msecs > lastTimeWrite + TIME_WRITE_PERIOD_MS) {
		lastTimeWrite = msecs;
		writeTimeToEEPROM();
	}
	
	updateButtonState();	
	if (stateFlags & BUTTON_STATE) {
		++buttonPressTime;
	}
	else buttonPressTime = 0;
	
	if (buttonPressTime == 1) {
		toggleState(USE_DIGITAL_DISPLAY_STATE);
	}
	else if (buttonPressTime == 6) {
		setState(SLEEP_STATE);
	}
	else if (stateFlags & SLEEP_STATE && (buttonPressTime == 3)) {
		clearState(SLEEP_STATE);
	}
}

void setup() {
	// Set LED pins as OUTPUT.
	// Anodes.
	DDRA |= 0xF8;
	DDRE |= 0x07;
	DDRC |= 0xFF;
	// Cathodes.
	DDRA |= 0x07;
	DDRD |= 0xFF;
	DDRB |= 0x1F;
	// Set LED pins LOW.
	clearAll();
	
	// Reset globals and update button state.
	usecs = 0;
	readTimeFromEEPROM();
	lastTimeWrite = msecs;
	
	stateFlags = USE_DIGITAL_DISPLAY_STATE;
	prevStateFlags = stateFlags;
	
	buttonPressTime = 0;
	updateButtonState();
	
	// Sleep enable with default idle mode.
	MCUCR |= (1 << SE);
	// Disable analog comparator.
	ACSR |= (1 << ACD);
	
	// Set 1/64 prescaler for Timer 1 for 125kHz frequency at 8Mhz.
	TCCR1B |= (1 << CS10) | (1 << CS11);
	// Enable Timer1 overflow interrupt.
	TIMSK |= (1 << TOIE1);
	// Enable interrupts.
	sei();
}

void powerDown() {
	cli();
	while (stateFlags & SLEEP_STATE) {
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
	}
	sei();
}

void writeTimeToEEPROM() {
	// Don't clear interrupt flag so that time stays relatively accurate.
	// Write process can fail, but it is updated often enough it doesn't
	// matter.
	//cli();
	eeprom_busy_wait();
	eeprom_write_dword((uint32_t*)TIME_ADDRESS, msecs);
	//sei();
}

void readTimeFromEEPROM() {
	eeprom_busy_wait();
	msecs = eeprom_read_dword((uint32_t*)TIME_ADDRESS);
}

static inline void debounce() {
	_delay_ms(DEBOUNCE_TIME_MS);
}

inline void setState(uint8_t flags) {
	prevStateFlags = stateFlags;
	stateFlags |= flags;
}

inline void clearState(uint8_t flags) {
	prevStateFlags = stateFlags;
	stateFlags &= ~flags;
}

inline void toggleState(uint8_t flags) {
	prevStateFlags = stateFlags;
	stateFlags ^= flags;
}

void updateButtonState() {
	// Make sure input pull-up is enabled. Makes visual glitches disappear
	// and button isn't self triggered.
	PORTD |= (1 << PD2);
	// Set button pin as input.
	DDRD &= ~(1 << PD2);
	// If button is pressed.
	if (~PIND & (1 << PD2))
		setState(BUTTON_STATE);
	else
		clearState(BUTTON_STATE);
	// Set button pin back to output.
	DDRD |= (1 << PD2);
}

void clearScreen() {
	for (uint8_t i = 0; i < ROWS; ++i) {
		screen[i] = 0;
	}
}

void renderDigitalClockOnScreen() {
	// Must be 32-bit, otherwise overflows at 18:18.
	uint32_t seconds = msecs / 1000;
	uint16_t minutes = seconds / 60;
	seconds %= 60;
	uint16_t hours = minutes / 60;
	minutes %= 60;
	
	// Second half of the top bar.
	screen[0] |= ~((1 << (7 - seconds)) - 1) & 0x00FF;	
	// First half of the top bar.
	if (seconds > 51)
		screen[0] |= ~((1 << (67 - seconds)) - 1);
	// Bottom bar.
	if (seconds > 21)
		screen[ROWS - 1] |= (1 << (seconds - 21)) - 1;
	// Right edge.
	if (seconds > 7) {
		uint16_t right_bar = ~((1 << (23 - seconds)) - 1);
		for (uint8_t i = 1; i < ROWS - 1; ++i) {
			screen[i] |= (right_bar >> (ROWS - i)) & 0x0001;
		}
	}
	// Left edge.
	if (seconds > 36) {
		uint16_t left_bar = (1 << (seconds - 36)) - 1;
		for (uint8_t i = 1; i < ROWS - 1; ++i) {
			screen[i] |= (left_bar << i) & 0x8000;
		}
	}

	// Display hours and minutes.
	for (uint8_t row = 0; row < DIGIT_ROWS; ++row) {
		// First digit of hours.
		screen[row + 2] |= 
			((digits[hours / 10] >> 
			((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 9;
		// Second digit of hours.
		screen[row + 2] |= 
			((digits[hours % 10] >> 
			((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 4;
		// First digit of minutes.
		screen[row + 9] |= 
			((digits[minutes / 10] >> 
			((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 9;
		// Second digit of minutes.
		screen[row + 9] |= 
			((digits[minutes % 10] >> 
			((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 4;
	}
}

// Bresenham's line algorithm
void drawLine(float x1, float y1, float x2, float y2)
{
	const uint8_t steep = (fabs(y2 - y1) > fabs(x2 - x1));
	float temp;
	if (steep) {
		temp = x1;
		x1 = y1;
		y1 = temp;

		temp = x2;
		x2 = y2;
		y2 = temp;
	}
	if (x1 > x2) {
		temp = x1;
		x1 = x2;
		x2 = temp;

		temp = y1;
		y1 = y2;
		y2 = temp;
	}

	const float dx = x2 - x1;
	const float dy = fabs(y2 - y1);

	float error = dx / 2.0f;
	const uint8_t ystep = (y1 < y2 ? 1 : -1);
	uint8_t y = (uint8_t)y1;

	const uint8_t maxX = (uint8_t)x2;

	for (uint8_t x = (uint8_t)x1; x < maxX; ++x)
	{
		if (steep)
			screen[x] |= (1 << (ROWS - y));		
		else
			screen[y] |= (1 << (ROWS - x));		
		error -= dy;
		if (error < 0) {
			y += ystep;
			error += dx;
		}
	}
}

void renderAnalogClockOnScreen()
{
	// Must be 32-bit, otherwise overflows at 18:18.
	uint32_t seconds = msecs / 1000;
	uint16_t minutes = seconds / 60;
	seconds %= 60;
	uint16_t hours = minutes / 60;
	minutes %= 60;
	
	const float x0 = 8.25f;
	const float y0 = 8.25f;

	// Hours
	float length = 5.f;
	float angle = 2.5f * PI - (hours / 12.f * 2.f * PI);
	float x1 = x0 + length * cos(angle);
	float y1 = y0 - length * sin(angle);
	drawLine(x0, y0, x1, y1);

	// Minutes
	length = 8.f;
	angle = 2.5f * PI - (minutes / 60.f * 2.f * PI);
	x1 = x0 + length * cos(angle);
	y1 = y0 - length * sin(angle);
	drawLine(x0, y0, x1, y1);

	// Seconds
	angle = 2.5f * PI - (seconds / 60.f * 2.f * PI);
	x1 = x0 + length * cos(angle);
	y1 = y0 - length * sin(angle);
	drawLine(x0, y0, x1, y1);
}

// Sets all LED output pins to LOW.
void clearAll() {
	// Cathodes.
	PORTA &= ~0xF8;
	PORTE &= ~0x07;
	PORTC &= ~0xFF;
	// Anodes.
	PORTA &= ~0x07;
	PORTD &= ~0xFF;
	PORTB &= ~0x1F;
}

// Sets all LEDs on.
void allOn() {
	// Cathodes.
	PORTA |= 0xF8;
	PORTE |= 0x07;
	PORTC |= 0xFF;
	// Anodes.
	PORTA &= ~0x07;
	PORTD &= ~0xFF;
	PORTB &= ~0x1F;
}

// Sets anode pins to LOW and cathode pins to HIGH.
// Used as an intermediate step when rendering line by line.
// NOTE: Has to be called before every line render. Even after clearAll().
void clearForDraw() {
	// Cathodes.
	PORTA &= ~0xF8;
	PORTE &= ~0x07;
	PORTC &= ~0xFF;
	// Anodes.
	PORTA |= 0x07;
	PORTD |= 0xFF;
	PORTB |= 0x1F;
}

// Takes number between 0-15 to activate certain row. Numbers over 15 are make
// undefined behavior.
void setRow(uint8_t row) {
	if (row < 5) {
		PORTA |= (1 << (row + PA3));
	}
	else if (row < 8) {
		PORTE |= (1 << (row - PA5));
	}
	else {
		PORTC |= (1 << (15 - row));
	}
}


void setColumn(uint16_t col) {
	for (int16_t i = 7; i >= 0; --i) {
		if (col & (1 << (2 * i))) {
			PORTD &= ~(1 << i);
		}
	}
	for (int16_t i = 2; i >= 0; --i) {
		if (col & (1 << (11 + 2*i))) {
			PORTA &= ~(1 << i);
		}
	}
	for (int16_t i = 0; i < 5; ++i) {
		if (col & (1 << (9 - 2*i))) {
			PORTB &= ~(1 << i);
		}
	}
}