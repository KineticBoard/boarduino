
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

static const uint8_t PROGMEM
	board_bmp[] = {
		B00000000,
		B00000000,
		B11111111,
		B01000010,
		B10100101,
		B01000010,
		B00000000,
		B00000000
	};

// global defined constants
#define MAX_STR_BUF 255
#define NunchukAddress 0x52
#define SerialBaudrate 19200
#define TWI_FREQ 400000L
#define PinMotor 10 // defines pin as a pwm variable voltage output

// button pressed/released constants
#define NunchukButtonPressed 0
#define NunchukButtonReleased 1

// structure being used to contain nunchuk data
struct nunchuk_data_s { 
	uint16_t joy_x;
	uint16_t joy_y;
	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;
	uint16_t button_z;
	uint16_t button_c;
};
nunchuk_data_s *nunchuk_curr = new nunchuk_data_s();
static boolean nunchuk_connected = false; // should be changed with nunchuk_set_connected(boolean);
static uint8_t nunchuk_data_same_detect = 0;
static uint8_t nunchuk_buf[6];
static uint8_t nunchuk_buf_prev[6];
static uint32_t nunchuk_message_count = 0;

static int motor_speed = 0; // forward speed from 0 to 255
static long motor_timestamp_last_calculation = 0;
static int motor_config = 0;

// settings
boolean debug = true;

// arduino setup function
void setup() {
	// setup serial connection
	if(debug) {
		Serial.begin(SerialBaudrate);
	}

	// setup matrix board
	matrix.begin(0x70);  // pass in the address

	// display board on matrix as welcome message, then show disconnected
	matrix_show_board();
	delay(1000);
	matrix_show_nunchuk_disconnected();

	// initialize motor
	motor_init();
	
	// initialise nunchuk
	nunchuk_init();

	//++ TODO(GeertJohan): delay? why?
	delay(100);
}

void loop() {
	// connect, if required
	if(!nunchuk_connected) {
		// try to connect with nunchuk
		nunchuk_connect();
	} else {
		// get new nunchuk data
		nunchuk_get_data();

		// verify nunchuk data
		nunchuk_check_data();

		// debug nunchuk data
		if(debug) {
			nunchuk_print_data();
		}
	}

	// check if still connected
	if(nunchuk_connected) {
		// calculate nunchuk fields
		nunchuk_update_data();
		
		// update motor speed with nunchuk data
		motor_calculate_speed();
	} else {
		// reset motor speed to 0
		motor_zero_speed();
	}

	// update motor output
	motor_write_update();
}

void matrix_show_board() {
	matrix.clear();
	matrix.drawBitmap(0, 0, board_bmp, 8, 8, LED_ON);
	matrix.writeDisplay();
}

void matrix_show_nunchuk_disconnected() {
	matrix.clear();
	matrix.drawLine(0,0, 7,7, LED_ON);
	matrix.drawLine(0,7, 7,0, LED_ON);
	matrix.writeDisplay();  // write the changes we just made to the display
}

void matrix_show_nunchuk_connected() {
	matrix.clear();
    matrix.setCursor(1,0);
    matrix.print("C");
    matrix.writeDisplay();
}

void matrix_show_bmp(const uint8_t *bmp) {
	matrix.clear();
	matrix.drawBitmap(0, 0, bmp, 8, 8, LED_ON);
	matrix.writeDisplay();
}

// static uint8_t matrix_stats[] = {
// 	B00000000,
// 	B00000000,
// 	B00000000,
// 	B00000000,
// 	B00000000,
// 	B00000000,
// 	B00000000,
// 	B00000000
// };
// void matrix_show_stats() {
// 	// nunchuk_curr->joy_y
// 	matrix_stats[0] = B11100000;

// 	// write to matrix
// 	matrix.clear();
// 	matrix.drawBitmap(0, 0, &matrix_stats[0], 8, 8, LED_ON);
// 	matrix.writeDisplay();
// }

// initialize nunchuk
void nunchuk_init() {
	// join i2c bus as master
	Wire.begin();
}

void nunchuk_set_connected(boolean cn) {
	if(cn) {
		nunchuk_connected = true;
		matrix_show_nunchuk_connected();
		if(debug) {
			Serial.println("nunchuk connected");
		}
	} else {
		nunchuk_connected = false;
		nunchuk_message_count = 0;
		matrix_show_nunchuk_disconnected();
		if(debug) {
			Serial.println("nunchuk disconnected");
		}
	}
}

// connect to nunchuk
void nunchuk_connect() {
	// indicate nunchuk connect starting
	if(debug) {
		Serial.println("nunchuk_connect() starting..");
	}

	// set registers for decrypted communication
	Wire.beginTransmission(NunchukAddress);
	Wire.write((uint8_t)0xF0);
	Wire.write((uint8_t)0x55);
	Wire.endTransmission();

	Wire.beginTransmission(NunchukAddress);
	Wire.write((uint8_t)0xFB);
	Wire.write((uint8_t)0x00);
	Wire.endTransmission();

	Wire.beginTransmission(NunchukAddress);
	Wire.write((uint8_t)0xFA);
	Wire.endTransmission();
	
	// request 6 bytes, then release bus.
	Wire.requestFrom(NunchukAddress, 6, true);
	
	delay(1); // doesn't seem to make a difference

	// char tmp_str_buf[MAX_STR_BUF];
	// int length = 0;
	// length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "-- Controller ident = [");
	
	// int cnt = 0;
	// while(Wire.available()) {
	// 	//if(debug) {
	// 	//  Serial.println("Wire available");
	// 	//}
	// 	nunchuk_buf[cnt] = Wire.read();
	// 	//if(debug) {
	// 	//  Serial.println("Wire read done..");
	// 	//}
	// 	if(cnt > 0) {
	// 		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, ", ");
	// 	}
	// 	if(cnt >= 6) {
	// 		if(debug) {
	// 			Serial.println("oops.??");
	// 		}
	// 	}
	// 	length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "%02X", nunchuk_buf[cnt]);
	// 	cnt++;
	// }
	// length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "]");

	// if(cnt == 6) {
	// 	initializing = false;
	// 	if(debug) {
	// 		Serial.println(tmp_str_buf);
	// 	}
	// } else {
	// 	if(debug) {
	// 		Serial.println("error: nunchuk init failed. retrying...");
	// 	}
	// 	delay(100);
	// }

	int readCount = 0;
	while(Wire.available()) {
		nunchuk_buf[readCount] = Wire.read();
		readCount++;
	}

	if(readCount == 6 && !nunchuk_detect_equal_data()) {
		// looks like we are connected again
		nunchuk_set_connected(true);
	} else {
		if(debug) {
			Serial.println("connecting nunchuk failed. retrying in 50ms...");
		}
		delay(50);
	}
}

// get raw data
void nunchuk_get_data() {
	// stash current data for later comparisno by nunchuk_check_data()
	nunchuk_buf_prev[0] = nunchuk_buf[0];
	nunchuk_buf_prev[1] = nunchuk_buf[1];
	nunchuk_buf_prev[2] = nunchuk_buf[2];
	nunchuk_buf_prev[3] = nunchuk_buf[3];
	nunchuk_buf_prev[4] = nunchuk_buf[4];
	nunchuk_buf_prev[5] = nunchuk_buf[5];

	// create counter to count received bytes (we must receive 6 bytes)
	int readCount = 0;
	
	// request 6 bytes from Wire
	Wire.requestFrom(NunchukAddress, 6);
	
	// read 6 bytes from Wire
	// TODO: limit to 6 loops??? Debug this with serial prints
	while(Wire.available()) {
		nunchuk_buf[readCount] = Wire.read();
		readCount++;
	}
	
	// send request tot nunchuk for next iteration
	Wire.beginTransmission(NunchukAddress);
	Wire.write((uint8_t)0x00);
	Wire.endTransmission();
	
	// check to have received 6 bytes
	if (readCount != 6) {
		nunchuk_set_connected(false);
		return;
	}
	
	// increment global sequence counter
	nunchuk_message_count++;
}

// check if data is valid
// detect disconnected nunchuk (exactly the same data)
void nunchuk_check_data() {
	if (nunchuk_buf[0] == 0x00 && nunchuk_buf[1] == 0x00 && 
			nunchuk_buf[2] == 0x00 && nunchuk_buf[3] == 0x00 && 
			nunchuk_buf[4] == 0x00 && nunchuk_buf[5] == 0x00) {
		if(debug) {
			Serial.println("error: nunchuk data pegged low (controller not synced?)");
		}
		delay(100); //++ TODO(GeertJohan): are these delays really that important??
		return;
	}

	if (nunchuk_buf[0] == 0xFF && nunchuk_buf[1] == 0xFF && 
			nunchuk_buf[2] == 0xFF && nunchuk_buf[3] == 0xFF && 
			nunchuk_buf[4] == 0xFF && nunchuk_buf[5] == 0xFF) {
		if(debug) {
			Serial.println("error: nunchuk data pegged high: attempting re-initialization");
		}
		nunchuk_set_connected(false);
		delay(100); //++ TODO(GeertJohan): are these delays really that important??
		return;
	}

	// assume disconnect when data is the same as last 3 times
	if (nunchuk_detect_equal_data()) {
		nunchuk_data_same_detect++;
		if(nunchuk_data_same_detect > 3) {
			if(debug) {
				Serial.println("error: data exactly the same three times in a row. Probably a empty battery");
			}
			nunchuk_set_connected(false);
		}
		delay(100); //++ TODO(GeertJohan): are these delays really that important??
		return;
	}

	// all fine :)
	nunchuk_data_same_detect = 0;
}

boolean nunchuk_detect_equal_data() {
	if (nunchuk_buf[0] == nunchuk_buf_prev[0] && nunchuk_buf[1] == nunchuk_buf_prev[1] &&
			nunchuk_buf[2] == nunchuk_buf_prev[2] && nunchuk_buf[3] == nunchuk_buf_prev[3] &&
			nunchuk_buf[4] == nunchuk_buf_prev[4] && nunchuk_buf[5] == nunchuk_buf_prev[5]) {
		return true;
	}
	return false;
}

// update data in useful structure
void nunchuk_update_data() {
	// calculate information values from byte array
	nunchuk_curr->joy_x    =   nunchuk_buf[0];
	nunchuk_curr->joy_y    =   nunchuk_buf[1];
	nunchuk_curr->button_z =  (nunchuk_buf[5] >> 0) & 1;
	nunchuk_curr->button_c =  (nunchuk_buf[5] >> 1) & 1;
	nunchuk_curr->accel_x  = ((nunchuk_buf[5] >> 2) & 3) | (nunchuk_buf[2] << 2);
	nunchuk_curr->accel_y  = ((nunchuk_buf[5] >> 4) & 3) | (nunchuk_buf[3] << 2);
	nunchuk_curr->accel_z  = ((nunchuk_buf[5] >> 6) & 3) | (nunchuk_buf[4] << 2);
}

void nunchuk_print_data() {
		char tmp_str_buf[MAX_STR_BUF];
		int length = 0;

		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
				"SEQ# %10d:  ", nunchuk_message_count);
		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
				"joy: (h,v)=(%3d, %3d)  ",
				nunchuk_curr->joy_x, nunchuk_curr->joy_y);
		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
				"accel: (x,y,z)=(%4d, %4d, %4d)  ",
				nunchuk_curr->accel_x, nunchuk_curr->accel_y, nunchuk_curr->accel_z);
		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
				"buttons: (c,z)=(%1d, %1d)  ",
				nunchuk_curr->button_c, nunchuk_curr->button_z);
		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
				"RAW: [%02X, %02X, %02X, %02X, %02X, %02X]",
				nunchuk_buf[0], nunchuk_buf[1], nunchuk_buf[2],
				nunchuk_buf[3], nunchuk_buf[4], nunchuk_buf[5]);
		length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "\n");
		
		if(debug) {
			Serial.print(tmp_str_buf);
		}
}

// initialize motor
void motor_init() {
	// set motorport to output
	pinMode(PinMotor, OUTPUT);
}

// zero motor speed
void motor_zero_speed() {
	motor_speed = 0;
	motor_config = 0;
}

// calculate motor speed from nunchuk values
void motor_calculate_speed() {
	if(nunchuk_curr->button_c == NunchukButtonPressed) {
		if(debug) {
			Serial.println("in config mode!");
		}
		if(nunchuk_curr->joy_y>159) {
			motor_config = 3;
		} else if(nunchuk_curr->joy_y<95) {
			motor_config = 1;
		} else {
			motor_config = 2;
		}
		return;
	}
	motor_config = 0;

	if(nunchuk_curr->button_z == NunchukButtonReleased) {
		motor_zero_speed();
		return;
	}


	// calculate stable accelValue
	int accelValue = map(nunchuk_curr->joy_y, 0, 255, -127, 127);
	if(accelValue < 3 && accelValue > -3) {
		accelValue = 0;
	}
	if(debug) {
		Serial.print("accelValue: ");
		Serial.println(accelValue);
	}

	int accelIncrement = map(accelValue, -127, 127, -5, 5);
	if(debug) {
		Serial.print("accelIncrement: ");
		Serial.println(accelIncrement);
	}

	// increment or decrement motor_speed
	motor_speed += accelIncrement;

	// check and fix motor_speed bounds
	// if(motor_speed < -255) {
	// 	motor_speed = -255;
	// }
	if(motor_speed < 0) {
		motor_speed = 0;
	}
	if(motor_speed > 255) {
		motor_speed = 255;
	}

	// sleep some time..
	long now = millis();
	long sleepMilis = 200 - (now - motor_timestamp_last_calculation);
	if(debug) {
		Serial.print("sleep ");
		Serial.print(sleepMilis);
		Serial.println("ms to ease acceleration");
	}
	if(sleepMilis > 0){
		delay(sleepMilis);
	}
	motor_timestamp_last_calculation = now;

	// write servoValue
	if(debug) {
		Serial.print("motor_speed: ");
		Serial.println(motor_speed);
	}
}

// update motor speed
void motor_write_update() {
	int motor_speed_pwm = 127;

	if(motor_config != 0) {
		motor_speed_pwm = map(motor_config, 1, 3, 1, 254);
	} else {
		// map motor speed to pwm value for "forward" when 
		if(motor_speed > 0){
			motor_speed_pwm = map(motor_speed, 0, 255, 138, 254);
			// motor_speed_pwm = map(motor_speed, -255, 255, 0, 255);
		}
	}
	if(debug) {
		Serial.print("motor_speed_pwm: ");
		Serial.println(motor_speed_pwm);
	}
	analogWrite(PinMotor, motor_speed_pwm);
}


// This might be very useful for later!! /GeertJohan
// Uses a trick found at http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_softreset
//#include <avr/wdt.h>
//void cmd_reset() {
//    if(debug) {
//      Serial.print("!! Reset requested\n\n");
//    }
//    mode_one_color_all(0,0,0);  // Blank the LED strip
//    delay(100);
//    cli();                 // Clear interrupts
//    wdt_enable(WDTO_15MS); // Set the watchdog to 15ms
//    while(1);              // Enter an infinite loop to trigger the watchdog
//}
