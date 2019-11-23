// MIT License
//
// Copyright (c) 2019 Elias Kauppi, Toivo Snåre
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <math.h>

// Row and column length.
#define ROWS 16

// Timing constants.
#define ROW_DELAY_US 1000
#define LED_DUTY_CYCLE_MAX 99
// With 16-bit timer and 1/64 prescaler on F_CPU.
#define USECS_PER_TIMER_OF 524288
#define DEBOUNCE_TIME_MS 20

// Rendering constants.
#define DIGIT_ROWS 5
#define DIGIT_COLUMNS 3
#define PI 3.14159265358979323846

// Time constants.
#define TIME_ADDRESS 0x100
#define TIME_WRITE_PERIOD_MS 30000UL
#define MILLISECONDS_PER_DAY 86400000UL

// Macro functions.
#define MIN(X, Y) (X < Y ? X : Y)
#define MAX(X, Y) (X < Y ? Y : X)

// State flags.
#define BUTTON_STATE 0x01
#define IDLE_STATE 0x02
#define USE_DIGITAL_DISPLAY_STATE 0x04
#define SETTINGS_STATE 0x08
#define SETTINGS_DIGITAL_STATE (SETTINGS_STATE | USE_DIGITAL_DISPLAY_STATE)

// Global interrupt controlled volatiles.
volatile uint_fast16_t usecs;
volatile uint_fast32_t msecs;
volatile uint_fast32_t lastTimeWrite;
volatile uint8_t stateFlags;
volatile uint8_t prevStateFlags;
volatile uint8_t buttonPressTime;
volatile uint8_t ledDutyCycle;

// Digits used in digital clock scaled to 5x3 matrix as bits.
const uint16_t digits[10] = {0x7B6F, 0x2492, 0x73E7, 0x73CF, 0x5BC9,
							 0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF};
// Rendering target and display source.
uint16_t screen[ROWS];

// Function declarations.
void setup();
void settings_mode();
void idleMode();
void writeTimeToEEPROM();
void readTimeFromEEPROM();
inline void setState(uint8_t);
inline void clearState(uint8_t);
inline void toggleState(uint8_t);
void updateButtonState();
void delayMicros(uint16_t);
void clearAll();
void clearForDraw();
void allOn();
void setRow(uint8_t);
void setColumns(uint16_t);
void renderDigitalClockOnScreen();
void renderDigitalTime(uint8_t);
void renderAnalogClockOnScreen();
inline void clearScreen();
void renderScreen();
void displayScreen();


int main() {
	// Setup all globals and micro-controller flags.
	setup();
	
	uint_fast32_t timing_counter = 0, button_time = 0;
	
	// Program loop.
	while (1) {
		// If sleep state flag is set, go idling.
		if (stateFlags & IDLE_STATE)
			idleMode();
		
		updateButtonState();
		
		// Render screen by predetermined bit state to use digital or analog
		// variant of the clock screen.
		renderScreen();
		displayScreen();
		_delay_ms(1);
		
		// Increment the timer and stop incrementing the button_time if the
		// button is pressed.
		++timing_counter;
		if (~stateFlags & BUTTON_STATE)
			button_time = timing_counter;
		
		// Toggle between digital and analog watch faces when 
		// button is released.
		if (~stateFlags & prevStateFlags & BUTTON_STATE)
			toggleState(USE_DIGITAL_DISPLAY_STATE);
		// If button is pressed for approximately 0.7 seconds,
		// set state for idling.
		else if (timing_counter == button_time + 30UL)
			setState(IDLE_STATE);
		// If button is pressed for approximately 1.5 seconds,
		// go to settings mode and after exiting, continue main program loop.
		else if (timing_counter == button_time + 60UL)
			settings_mode();
	}
}

// Timed interrupt which increments the clock and checks for
// button press. Writes current time periodically to EEPROM.
ISR(TIMER1_OVF_vect) {
	usecs += 531;
	msecs += 529 + usecs / 1000;
	usecs %= 1000;
	// Reset after 24 hours.
	msecs %= MILLISECONDS_PER_DAY;
	
	// Write current time to EEPROM periodically.
	if (msecs > lastTimeWrite + TIME_WRITE_PERIOD_MS
		|| msecs < lastTimeWrite) {
		lastTimeWrite = msecs;
		writeTimeToEEPROM();
	}
	
	updateButtonState();	
	// Increment press time.
	++buttonPressTime;
	// Reset if not pressed.
	if (~stateFlags & BUTTON_STATE)
		buttonPressTime = 0;

	if (stateFlags & IDLE_STATE && (buttonPressTime == 3))
		clearState(IDLE_STATE);
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
	ledDutyCycle = LED_DUTY_CYCLE_MAX;
	
	// Reset globals and update button state.
	usecs = 0;
	readTimeFromEEPROM();
	lastTimeWrite = msecs;
	
	// Preset digital display after start-up.
	stateFlags = USE_DIGITAL_DISPLAY_STATE;
	prevStateFlags = stateFlags;
	
	// Reset button press timer and update button current state.
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

void settings_mode() {
	// Disable interrupts for the whole process since time is set here
	// therefore no updating is needed.
	cli();	
	setState(SETTINGS_STATE);
	
	// Overflows approximately after 497 days.
	// -> Hard reset required.
	// Settings mode for 1 year and 132 days? Batteries don't last
	// that long.
	uint_fast32_t timing_counter = 0, blink_time = 0,
				  button_time = 0;
	uint8_t time_flag = 0x01;
	uint8_t blink_flag = 0x00;
	// Used to ignore multiple triggers.
	uint8_t setting_trigger_flag = 0x00;
	
	while (stateFlags & SETTINGS_STATE) {
		if (timing_counter > blink_time + 25UL) {
			blink_time = timing_counter;
			blink_flag ^= 0xFF;
		}	
		
		if (~stateFlags & prevStateFlags & BUTTON_STATE &&
			~setting_trigger_flag & 0x01) {
			// Reset seconds.
			msecs -= msecs % 60000UL;
			// One minute.
			if (time_flag & 0x01)
				msecs += 60000UL;
			// One hour.
			else if (time_flag & 0x04)
				msecs += 3600000UL;
			// Whole day -> reset to 0.
			msecs %= MILLISECONDS_PER_DAY;
			
			setting_trigger_flag |= 0x01;
		}
		else if (timing_counter > button_time + 75UL && 
				 ~setting_trigger_flag & 0x02) {
			time_flag ^= 0x05;
			setting_trigger_flag |= 0x02;
		}
		else if (timing_counter > button_time + 200UL)
			clearState(SETTINGS_STATE);
		
		updateButtonState();
		
		clearScreen();
		renderDigitalTime(time_flag & blink_flag);
		displayScreen();
		_delay_ms(1);
		
		// Set button press start.
		++timing_counter;
		if (~stateFlags & prevStateFlags & BUTTON_STATE)
			setting_trigger_flag = 0;
		else if (~stateFlags & BUTTON_STATE)
			button_time = timing_counter;
	}
	// Write time to EEPROM before continuing.
	writeTimeToEEPROM();
	
	// Turn interrupts back on.
	sei();
}

void idleMode() {
	cli();
	while (stateFlags & IDLE_STATE) {
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

// Delays at least time specified. If interrupt occurs during waiting, time
// waited is interrupt + delay loop. Isn't very accurate.
void delayMicros(uint16_t us) {
	_delay_loop_2(2 * us - 1);
}

inline void clearScreen() {
	for (uint8_t i = 0; i < ROWS; ++i)
		screen[i] = 0;
}

void renderScreen() {
	// Reset render vector.
	clearScreen();
	
	// Choose algorithm to render.
	if (stateFlags & USE_DIGITAL_DISPLAY_STATE)
		renderDigitalClockOnScreen();
	else
		renderAnalogClockOnScreen();
}

void displayScreen() {
	// Display screen with preset brightness.
	for (uint8_t i = 0; i < ROWS; ++i) {
		clearForDraw();
		delayMicros(ROW_DELAY_US / 100 * (100 - ledDutyCycle));
		setRow(i);
		setColumns(screen[i]);
		delayMicros(ROW_DELAY_US / 100 * ledDutyCycle);
	}
	// Prevent the last row blasting at full brightness.
	clearForDraw();
}

void renderDigitalClockOnScreen() {
	// Must be 32-bit, otherwise overflows at 18:12.
	uint32_t seconds = msecs / 1000UL;
	seconds %= 60UL;	
	
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
	renderDigitalTime(0);
}

void renderDigitalTime(uint8_t hideFlag) {
	uint16_t minutes = msecs / 60000UL;
	uint16_t hours = minutes / 60;
	minutes %= 60;
	
	// Display hours and minutes.
	for (uint8_t row = 0; row < DIGIT_ROWS; ++row) {
		if (~hideFlag & 0x08) {
			// First digit of hours.
			screen[row + 2] |=
				((digits[hours / 10] >>
				((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 9;
		}
		if (~hideFlag & 0x04) {
			// Second digit of hours.
			screen[row + 2] |=
				((digits[hours % 10] >>
				((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 4;
		}
		if (~hideFlag & 0x02) {
			// First digit of minutes.
			screen[row + 9] |=
				((digits[minutes / 10] >>
				((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 9;
		}
		if (~hideFlag & 0x01) {
			// Second digit of minutes.
			screen[row + 9] |=
				((digits[minutes % 10] >>
				((DIGIT_ROWS - row - 1) * DIGIT_COLUMNS)) & 7) << 4;
		}
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
	// Must be 32-bit, otherwise overflows at 18:12.
	uint32_t seconds = msecs / 1000UL;
	uint16_t minutes = seconds / 60UL;
	seconds %= 60UL;
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
	if (row < 5)
		PORTA |= (1 << (row + PA3));
	else if (row < 8)
		PORTE |= (1 << (row - PA5));
	else
		PORTC |= (1 << (15 - row));
}

// Takes 16-bit value where each bit is represents a column in order from left
// to right. setRow is used to activate certain row where LEDs are 
// to be activated.
void setColumns(uint16_t cols) {
	int8_t i;
	for (i = 7; i >= 0; --i) {
		if (cols & (1 << (2 * i)))
			PORTD &= ~(1 << i);
	}
	for (i = 2; i >= 0; --i) {
		if (cols & (1 << (11 + 2*i)))
			PORTA &= ~(1 << i);
	}
	for (i = 0; i < 5; ++i) {
		if (cols & (1 << (9 - 2*i)))
			PORTB &= ~(1 << i);
	}
}