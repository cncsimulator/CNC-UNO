/******************************************************************************************
* CNC UNO Firmvare Ver 1.0 Beta.
******************************************************************************************
* Copyright (c) ChrisE 2015
*
* This file is free software; you can redistribute it and/or modify
* it under the terms of either the GNU General Public License version 2
* or the GNU Lesser General Public License version 2.1, both as
* published by the Free Software Foundation.
******************************************************************************************
* The CNC UNO is a DIY small desktop CNC machine made from
* wood and 3D printed parts. The machine is controlled by an Arduino Mega 2560.
* For more information, see project at Instructables.com:
* For simulation and communication software please visit www.cncsimulator.com
*
*****************************************************************************************/

#include "stepper.h"
#include "LCD_Stuff.h"
#include <math.h>
#include <avr/eeprom.h>
#include "switch.h"
#include "EncoderPolling.h"
#include <sd.h>

#define ENCODER_USE_INTERRUPTS

#define PIN_SCE   4
#define PIN_RESET 5
#define PIN_DC    3
#define PIN_SDIN  2
#define PIN_SCLK  14

#define encoderPin1 6
#define encoderPin2 7

#define JOY_X    A1
#define JOY_Y    A2
#define POT      A0

#define SPINDLEPIN	22

#define BTN1 8
#define BTN2 9
#define BTN3 10

#define LEDGREEN 11
#define LEDYEL 13
#define LEDRED 12

#define XSTOP A9
#define YSTOP A10
#define ZSTOP A8

#define XAXIS_STEP_PER_REV  200
#define YAXIS_STEP_PER_REV  200  
#define ZAXIS_STEP_PER_REV  200

#define MM_PER_STEP_X 0.2
#define MM_PER_STEP_Y 0.2
#define MM_PER_STEP_Z 0.01

#define MM_PER_REV_X (MM_PER_STEP_X * XAXIS_STEP_PER_REV)
#define MM_PER_REV_Y (MM_PER_STEP_Y * YAXIS_STEP_PER_REV)
#define MM_PER_REV_Z (MM_PER_STEP_Z * ZAXIS_STEP_PER_REV)

#define XSTEPS_MAX 700
#define YSTEPS_MAX 1000
#define ZSTEPS_MAX 6300

#define TOOLCHANGEPOS_X 0
#define TOOLCHANGEPOS_Y YSTEPS_MAX
#define TOOLCHANGEPOS_Z ZSTEPS_MAX

#define XY_MAX_FEED 55
#define Z_MAX_FEED 60

#define HOMESPEED 30
#define Z_HOMESPEED 60

#define XY_FAST_FEED 35
#define Z_FAST_FEED Z_MAX_FEED

#define XJOG_STEPS 2
#define YJOG_STEPS 2
#define ZJOG_STEPS 10

#define RELAXTIMEOUT 10000

#define DNULL -99999999

#define CNCBUF_SIZE 5000

bool jogging;

int xZeroSteps =0;
int yZeroSteps =0;
int zZeroSteps =0;

int lastJogAxis =0;	// 1 = X  2 = Y  3 = Z

// Setup stepper motors
Stepper xstepper(XAXIS_STEP_PER_REV,37,35,33,31);
Stepper ystepper(YAXIS_STEP_PER_REV,43,45,39,41);
Stepper zstepper(ZAXIS_STEP_PER_REV,47,49,27,29);

int xCurrentSteps = 0;
int yCurrentSteps = 0;
int zCurrentSteps = 0;

int xGoalSteps = 0;
int yGoalSteps = 0;
int zGoalSteps = 0;

// Setup switches
Switch b1 = Switch(BTN1, INPUT_PULLUP,HIGH,1);
Switch b2 = Switch(BTN2, INPUT_PULLUP,HIGH,1);
Switch b3 = Switch(BTN3, INPUT_PULLUP,HIGH,1);

double feedoverride;

char string[20];
String theString;

int feedxy = 20;
int feedz = Z_MAX_FEED;
int pfxy;
int pfz;

boolean alarm;
boolean relaxed;

int xjoy;
int yjoy;
int pot;
int last_pot;

char serialbuf[256];
int serialbufpos=0;
unsigned long m;

boolean XZ_jog;
boolean paused = false;
boolean toolChange = false;
int changeToTool =0;


unsigned long milliseconds;

int lcd_xaxis = -1;  // For the LCD
int lcd_yaxis = -1;
int lcd_zaxis = -1;

int current_mnu = 0;	


void LcdPrint(String text)
{
	text.toCharArray(string, text.length()+1);
	LcdString(string);
}

void LcdPrint(double num)
{
	char buf[10];
	dtostrf(num,10,2,buf);
	LcdPrint(String(buf));
}

void LcdCharacter(char character)
{
	LcdWrite(LCD_D, 0x00);
	for (int index = 0; index < 5; index++)
	{
		LcdWrite(LCD_D, ASCII[character - 0x20][index]);
	}
	LcdWrite(LCD_D, 0x00);
}

void LcdClear(void)
{
	for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
	{
		LcdWrite(LCD_D, 0x00);
	}
	LcdWrite( 0, 0x80 | 0);  // Column.
	LcdWrite( 0, 0x40 | 0);  // Row.  
}

void LcdInitialise(void)
{
	pinMode(PIN_SCE, OUTPUT);
	pinMode(PIN_RESET, OUTPUT);
	pinMode(PIN_DC, OUTPUT);
	pinMode(PIN_SDIN, OUTPUT);
	pinMode(PIN_SCLK, OUTPUT);
	digitalWrite(PIN_RESET, LOW);
	digitalWrite(PIN_RESET, HIGH);
	LcdWrite(LCD_C, 0x21 );  // LCD Extended Commands.
	LcdWrite(LCD_C, 0xB0 );  // Set LCD Vop (Contrast). 
	LcdWrite(LCD_C, 0x04 );  // Set Temp coefficent. //0x04
	LcdWrite(LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
	LcdWrite(LCD_C, 0x0C );  // LCD in normal mode.
	LcdWrite(LCD_C, 0x20 );
	LcdWrite(LCD_C, 0x0C );
}

void LcdString(char *characters)
{
	while (*characters)
		LcdCharacter(*characters++);
}

void LcdWrite(byte dc, byte data)
{
	digitalWrite(PIN_DC, dc);
	digitalWrite(PIN_SCE, LOW);
	shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
	digitalWrite(PIN_SCE, HIGH);
}

void LcdPos(int x, int y)
{
	LcdWrite( 0, 0x80 | x);  // Column.
	LcdWrite( 0, 0x40 | y);  // Row.  
}

void Relax()
{
	// Turn off power to all steppers
	// to save power and prevent overheating.
	digitalWrite(31,0);
	digitalWrite(33,0);
	digitalWrite(35,0);
	digitalWrite(37,0);

	digitalWrite(39,0);
	digitalWrite(41,0);
	digitalWrite(43,0);
	digitalWrite(45,0);

	digitalWrite(47,0);
	digitalWrite(49,0);
	digitalWrite(51,0);
	digitalWrite(53,0);  

	relaxed = true;
}

void Home()
{

	digitalWrite(LEDRED,1);
	digitalWrite(LEDGREEN,0);

	xstepper.setSpeed(HOMESPEED);
	ystepper.setSpeed(HOMESPEED);
	zstepper.setSpeed(Z_HOMESPEED);

	while(digitalRead(ZSTOP)==1)
		zstepper.step(1);

	zstepper.step(-200);

	while(digitalRead(XSTOP)==1)
		xstepper.step(-1);

	xstepper.step(10);

	while(digitalRead(YSTOP)==1)
		ystepper.step(-1);

	ystepper.step(10);

	xCurrentSteps = 0;
	yCurrentSteps = 0;
	zCurrentSteps = ZSTEPS_MAX;

	digitalWrite(SPINDLEPIN, 1);
	delay(100);
	digitalWrite(SPINDLEPIN, 0);

	relaxed = false;

	xstepper.setSpeed(XY_FAST_FEED);
	ystepper.setSpeed(XY_FAST_FEED);
	zstepper.setSpeed(Z_FAST_FEED);

}

void Demo()
{
	Home();
	SetFeed(XY_FAST_FEED, Z_FAST_FEED);

	while(1 && !alarm)
	{
		gotoStepXYZ(1,1,40/MM_PER_STEP_Z);
		gotoStepXYZ(500,500,ZSTEPS_MAX-1);
		gotoStepXYZ(1,500,40 / MM_PER_STEP_Z);

		digitalWrite(SPINDLEPIN, 1);
		delay(1000);
		digitalWrite(SPINDLEPIN, 0);

		gotoStepXYZ(1,1,ZSTEPS_MAX-1);

		if(BTN(1))
		{
			Relax();
			return;
		}
	}
}

void Alarm(String Error)
{
	Relax();
	LcdClear();

	LcdPrint("ALARM:");
	LcdPos(0,1);
	LcdPrint(Error);
	LcdPos(0,5);
	LcdPrint("RESET");

	Serial.println("Alarm!");
	Serial.println(Error);

	xCurrentSteps = 0;
	yCurrentSteps = 0;
	zCurrentSteps = 0;

	alarm=true;

	while(1)
	{
		digitalWrite(LEDRED,1);
		delay(100);
		if(BTN(1))
		{
			digitalWrite(LEDRED,0);
			LcdClear();
			delay(10);
			while(BTN(1))
				delay(10);
			return;
		}        

		digitalWrite(LEDRED,0);
		delay(100); 
		if(BTN(1))
		{
			LcdClear();
			delay(10);
			while(BTN(1))
				delay(10);
			return;
		}        
	}
}

bool BTN(int num)
{
	switch(num)
	{
	case 1:
		b1.poll();
		return b1.pushed();
	case 2:
		b2.poll();
		return b2.pushed();
	case 3:
		b3.poll();
		return b3.pushed();      
	}
	return false;       
}

void gotoStepX(long x)
{
	stepaxis(&xstepper, x- xCurrentSteps);
	xCurrentSteps = x;
}

void gotoStepY(long y)
{
	stepaxis(&ystepper, y-yCurrentSteps);
	yCurrentSteps = y;
}

void gotoStepZ(long z)
{
	stepaxis(&zstepper, z-zCurrentSteps);
	zCurrentSteps = z;
}

void checkLimitsAndStops()
{
	if(digitalRead(XSTOP)==0)
	{
		delay(100);
		if(digitalRead(XSTOP)==0)		
			Alarm("XSTOP");
	}
	if(digitalRead(YSTOP)==0)
	{
		delay(100);
		if(digitalRead(YSTOP)==0)
			Alarm("YSTOP");
	}
	if(digitalRead(ZSTOP)==0)
	{
		delay(100);
		if(digitalRead(ZSTOP)==0)
			Alarm("ZSTOP");
	}

	if(xCurrentSteps>XSTEPS_MAX)
		Alarm("XMAX");
	if(yCurrentSteps>YSTEPS_MAX)
		Alarm("YMAX");
	if(zCurrentSteps>ZSTEPS_MAX)
		Alarm("ZMAX");
	if(xCurrentSteps<0)
		Alarm("X<0");
	if(yCurrentSteps<0)
		Alarm("Y<0");
	if(zCurrentSteps<0)
		Alarm("Z<0");
}


void gotoStepXYZ(int x, int y, int z)
{
	double shortStep = 0;

	double xdiff = abs(x-xCurrentSteps);
	double ydiff = abs(y-yCurrentSteps);
	double zdiff = abs(z-zCurrentSteps);

	// If only x-axis, call single axis function
	if(xdiff!=0 && ydiff==0 && zdiff==0)
	{
		gotoStepX(x);
		return;
	}
	// If only y-axis, call single axis function
	if(ydiff!=0 && xdiff==0 && zdiff==0)
	{
		gotoStepY(y);
		return;
	}
	// If only z-axis, call single axis function
	if(zdiff!=0 && ydiff==0 && xdiff==0)
	{
		gotoStepZ(z);
		return;
	}

	double xstep, ystep, zstep;
	double currx = xCurrentSteps;
	double curry = yCurrentSteps;
	double currz = zCurrentSteps;

	if(xdiff >= ydiff && xdiff > zdiff)
	{
		xstep = 1;
		ystep = abs(ydiff/xdiff);   
		zstep = abs(zdiff/xdiff);
	}
	else if(ydiff >= xdiff && ydiff > zdiff)
	{
		ystep = 1;  
		xstep = abs(xdiff/ydiff);
		zstep = abs(zdiff/ydiff);
	}  
	else
	{
		zstep = 1;
		xstep = abs(xdiff/zdiff);
		ystep = abs(ydiff/zdiff);   
	}

	if(x < xCurrentSteps)
		xstep=-xstep;

	if(y < yCurrentSteps)
		ystep=-ystep;

	if(z < zCurrentSteps)
		zstep=-zstep;

	digitalWrite(LEDRED,1);
	digitalWrite(LEDGREEN,0);

	if(abs(xstep)==1)
	{
		while(xCurrentSteps!=x && !alarm)
		{
			currx+=xstep;
			curry+=ystep;
			currz+=zstep;

			gotoStepX(currx);
			gotoStepY(curry);
			gotoStepZ(currz);
		}
	}
	else if(abs(ystep)==1)
	{
		while(yCurrentSteps!=y && !alarm)
		{
			currx+=xstep;
			curry+=ystep;
			currz+=zstep;

			gotoStepX(currx);
			gotoStepY(curry);
			gotoStepZ(currz);
		}
	} 
	else
	{
		while(zCurrentSteps!=z && !alarm)
		{
			currx+=xstep;
			curry+=ystep;
			currz+=zstep;

			gotoStepX(currx);
			gotoStepY(curry);
			gotoStepZ(currz);

		}
	}

	digitalWrite(LEDRED,0);
	digitalWrite(LEDGREEN,1);

}

void SetFeed(int f, int fz)
{
	pfxy = f;
	pfz = fz;
}

void Dwell(int millisecs)
{
	delay(millisecs);
	milliseconds = millis(); // Reset relax-conunter
}

void goClockwise(double x, double y, double i, double j)
{
	double xc = xCurrentSteps + i;
	double yc = yCurrentSteps + j;

	double a1 = AnglePP(xc,yc,xCurrentSteps, yCurrentSteps);
	double a2 = AnglePP(xc,yc,x,y);
	double r=DistPP(xc,yc,x,y);

	a2 = a2_clockwise(a1,a2);

	double xnow, ynow;

	digitalWrite(LEDRED,1);
	digitalWrite(LEDGREEN,0);

	for(double a = a1; a>a2; a-= PI/180)
	{
		xnow = xc + r * cos(a);
		ynow = yc + r * sin(a);      
		gotoStepX(xnow);
		gotoStepY(ynow);
	}

	digitalWrite(LEDRED,0);
	digitalWrite(LEDGREEN,1);

}

void goAntiClockwise(double x, double y, double i, double j)
{
	double xc = xCurrentSteps + i;
	double yc = yCurrentSteps + j;

	double a1 = AnglePP(xc,yc,xCurrentSteps, yCurrentSteps);
	double a2 = AnglePP(xc,yc,x,y);

	double r=DistPP(xc,yc,x,y);

	a2 = a2_anticlockwise(a1,a2);

	double xnow, ynow;

	digitalWrite(LEDRED,1);
	digitalWrite(LEDGREEN,0);


	for(double a = a1; a<a2; a+= PI/180)
	{
		xnow = xc + r * cos(a);
		ynow = yc + r * sin(a);      
		gotoStepXYZ(xnow, ynow, zCurrentSteps);
	}

	digitalWrite(LEDRED,0);
	digitalWrite(LEDGREEN,1);

}

double AnglePP(double x1, double y1, double x2, double y2)
{
	double v;

	if (x1 == x2 && y1 == y2) return (0);
	if (abs(x1 - x2) > abs(y1 - y2))
	{
		v = atan((y2 - y1) / (x2 - x1));
		if (x2 < x1) v += PI;
	}
	else
	{
		v = PI / 2 - atan((x2 - x1) / (y2 - y1));
		if (y2 < y1) v += PI;
	}
	return (v);
}

double a2_clockwise(double a1, double a2)
{
	while (a2 < a1) a2 += (PI * 2);
	while (a2 > a1 - 1e-15) a2 -= (PI * 2);
	return a2;
}

double a2_anticlockwise(double a1, double a2)
{
	while (a2 > a1) a2 -= PI * 2;
	while (a2 < a1 + 1e-15) a2 += PI * 2;
	return (a2);
}

double DistPP(double x1, double y1, double x2, double y2)
{
	x1 -= x2;
	y1 -= y2;
	return (sqrt(x1 * x1 + y1 * y1));
}

void WelcomeScreen()
{
	digitalWrite(LEDGREEN, 1);
	digitalWrite(LEDYEL, 1);
	digitalWrite(LEDRED, 1); 

	LcdInitialise();
	LcdClear();
	LcdString("* CNC UNO *");
	LcdPos(0,1);
	LcdString("Ver 1.0");
	LcdPos(0,2);
	LcdString("CNCSimulator");
	LcdPos(0,3);
	LcdString(".com 2015");
	delay(2000);  

	digitalWrite(LEDGREEN, 0);
	digitalWrite(LEDYEL, 0);
	digitalWrite(LEDRED, 0);

}

void setup(void)
{  

	pinMode(53, OUTPUT);	// For the SD module

	while (!eeprom_is_ready());
	cli();

	// Read zero position
	xZeroSteps = eeprom_read_word((uint16_t*)0);
	yZeroSteps = eeprom_read_word((uint16_t*)2);
	zZeroSteps = eeprom_read_word((uint16_t*)4);

	sei();

	pinMode(LEDGREEN, OUTPUT);
	pinMode(LEDYEL, OUTPUT);
	pinMode(LEDRED, OUTPUT);
	pinMode(SPINDLEPIN, OUTPUT);

	WelcomeScreen();

	Serial.begin(115200);

	pinMode(BTN1, INPUT);
	pinMode(BTN2, INPUT);
	pinMode(BTN3, INPUT);

	digitalWrite(BTN1, 1); // Pullup
	digitalWrite(BTN2, 1); // Pullup
	digitalWrite(BTN3, 1); // Pullup

	pinMode(XSTOP, INPUT_PULLUP);
	pinMode(YSTOP, INPUT_PULLUP);
	pinMode(ZSTOP, INPUT_PULLUP);

	Home();

	milliseconds = millis();

	encoder_begin();  // Start the library
	attach_encoder(0, encoderPin1, encoderPin2);  // Attach an encoder to pins A and B
	Serial.write(17);  // XON

	pfxy = XY_MAX_FEED;
	pfz = Z_MAX_FEED;
}

void PrintAxis()
{
	char dblStr[12];
	if(lcd_xaxis != xCurrentSteps || lcd_yaxis != yCurrentSteps || lcd_zaxis != zCurrentSteps)
	{
		LcdClear();

		dtostrf((xCurrentSteps-xZeroSteps)*MM_PER_STEP_X, 3,2, dblStr);
		LcdPrint(String("X: ")+dblStr);
		LcdPos(0,1);
		dtostrf((yCurrentSteps-yZeroSteps)*MM_PER_STEP_Y, 3,2, dblStr);
		LcdPrint(String("Y: ")+dblStr);
		LcdPos(0,2);
		dtostrf((zCurrentSteps-zZeroSteps)*MM_PER_STEP_Z, 3,2, dblStr);
		LcdPrint(String("Z: ")+dblStr);
		LcdPos(0,3);
		dtostrf((feedoverride * (double)feedxy), 2,0,dblStr);
		LcdPrint(String("Feed:")+dblStr);

		lcd_xaxis = xCurrentSteps;
		lcd_yaxis = yCurrentSteps;
		lcd_zaxis = zCurrentSteps;
	}

}

void runtFromSD()
{
	if (!SD.begin(53)) 
	{
		Alarm("SD init fail");
		return;
	}

	File cncFile = SD.open("RUN.CNC");
	if(cncFile)
	{
		while(cncFile.available())
		{
			handleButtons();

			if(paused)
			{
				handleMenu();
				continue;
			}

			serialbuf[serialbufpos]=(char)cncFile.read();
			if(serialbuf[serialbufpos]==13)	// Line complete
			{
				serialbuf[serialbufpos+1]=0;
				Interpret(String(serialbuf));

				serialbufpos=0;
				if(cncFile.available()  > 0 && !paused)
				{
					LcdPos(0,5);
					LcdPrint("       PAUSE");
				}
			}
			else
				serialbufpos++;
		}


		cncFile.close();
	}
	else
		Alarm("RUN.CNC missing");

}

void ZeroAll()
{
	// Write current pos as zeropos in eeprom
	xZeroSteps = xCurrentSteps;
	yZeroSteps = yCurrentSteps;
	zZeroSteps = zCurrentSteps;

	while (!eeprom_is_ready());
	cli();
	eeprom_write_word((uint16_t*)0,xZeroSteps);
	eeprom_write_word((uint16_t*)2,yZeroSteps);
	eeprom_write_word((uint16_t*)4,zZeroSteps);
	sei();

	LcdClear();
	LcdPos(0,0);
	LcdPrint("ZERO SET!");
	delay(500);

}

void handleMenu()
{
	LcdPos(0,5);

	if(paused)
	{
		LcdPrint("UNPAUSE     ");

		if(toolChange)
		{
			char tstr[5];

			LcdPos(0,4);
			itoa(changeToTool,tstr,10);
			LcdPrint(String("SET TOOL ")+tstr);
		}

		return;
	}

	switch (current_mnu)
	{
	case 0:
		if(!XZ_jog)
			LcdPrint("HOME SPNDL >");
		else
			LcdPrint("HOME SPNDL >");
		break;
	case 1:
		if(!XZ_jog)
			LcdPrint("ZERO JOGXY >");
		else
			LcdPrint("ZERO JOGXZ >");
		break;
	case 2:
		LcdPrint("DEMO ABOUT >");
		break;
	case 3:
		LcdPrint("RUN PAUSE  >");
		break;
	}

}

void handleButtons()
{
	int numMenus = 4;

	if(paused)
	{
		if(BTN(1))
		{
			paused = false;
			toolChange = false;
		}
		return;
	}

	switch (current_mnu)
	{
	case 0:
		if(BTN(1))
			Home();
		else if(BTN(2))
			digitalWrite(SPINDLEPIN, !digitalRead(SPINDLEPIN));
		else if(BTN(3))
		{
			current_mnu++;
			if(current_mnu==numMenus)
				current_mnu=0;
		}
		break;
	case 1:
		if(BTN(1))
			ZeroAll();
		else if(BTN(2))
			XZ_jog=!XZ_jog;
		else if(BTN(3))
		{
			current_mnu++;
			if(current_mnu==numMenus)
				current_mnu=0;
		}
		break;
	case 2:
		if(BTN(1))
			Demo();
		else if(BTN(2))
		{
			WelcomeScreen();
			delay(2000);
		}
		else if(BTN(3))
		{
			current_mnu++;
			if(current_mnu==numMenus)
				current_mnu=0;
		}
		break;
	case 3:
		if(BTN(1))
			runtFromSD();
		else if(BTN(2))
			paused  = true;
		else if(BTN(3))
		{
			current_mnu++;
			if(current_mnu==numMenus)
				current_mnu=0;
		}
		break;
	}

}


// The main loop
void loop(void)
{ 
	digitalWrite(LEDRED,0);
	digitalWrite(LEDGREEN,1);

	// Rotary encoder part
	int dir = encoder_data(0);
	if(dir!=0)
	{
		switch (lastJogAxis)
		{
		case 1:
			stepaxis(&xstepper, dir);
			xCurrentSteps+=dir;
			break;
		case 2:
			stepaxis(&ystepper, dir);
			yCurrentSteps+=dir;
			break;
		case 3:
			stepaxis(&zstepper,dir);
			zCurrentSteps+=dir;
			break;
		default:
			break;
		}
	}

	if(millis() - milliseconds> RELAXTIMEOUT)
	{
		if(!paused)
			Relax();
		milliseconds = millis();
	}

	if(millis()>m)
	{
		PrintAxis();
		m=millis()+500;
	}

	handleMenu();

	doJogging();
	handleButtons();

	if(alarm)
		alarm = false;    

	if(!paused)
	{

		while(Serial.available()  > 0)
		{
			Serial.write(19);  // XOFF

			if(digitalRead(BTN3)==0)
			{
				paused = true;
				delay(500);
				while (digitalRead(BTN3)==0);
				delay(100);

				break;
			}
			digitalWrite(LEDYEL,1);
			serialbuf[serialbufpos]=(char)Serial.read();
			if(serialbuf[serialbufpos]==13)	// Line complete
			{
				Serial.write(19);  // XOFF
				Serial.write("OK\r") ;


				serialbuf[serialbufpos+1]=0;
				Serial.print(serialbuf);
				Interpret(String(serialbuf));
				if(Serial.available()  > 0 && !paused)
				{
					LcdPos(0,5);
					LcdPrint("       PAUSE");
				}
				serialbufpos=0;
				if(paused)
					break;
			}
			else
				serialbufpos++;
		}

		Serial.write(17);  // XON

		digitalWrite(LEDYEL,0);
	}
}

void Interpret(String block)
{
	int xstep, ystep, zstep;
	double x = DNULL,y = DNULL,z = DNULL;
	double CCI;
	double CCJ;

	if(block.indexOf("F")!=-1)
	{
		double f = findValue(block, "F");
		if(f!=DNULL)
		{
			feedxy = f / MM_PER_REV_X;
			feedz = f / MM_PER_REV_Z;
		}
	}

	if(block.indexOf("M03")!=-1)
	{
		digitalWrite(SPINDLEPIN,1);
		delay(500);
	}
	else if(block.indexOf("M04")!=-1)
	{
		digitalWrite(SPINDLEPIN,1);
		delay(500);
	}
	else if(block.indexOf("M05")!=-1)
		digitalWrite(SPINDLEPIN,0);
	else if(block.indexOf("M30")!=-1)
		digitalWrite(SPINDLEPIN,0);
	else if(block.indexOf("M01")!=-1)
		paused = true;



	if(block.indexOf("G00")!=-1)
	{
		x=findValue(block,"X");
		y=findValue(block,"Y");
		z=findValue(block,"Z");

		if(x!=DNULL)
			xstep = (x / MM_PER_STEP_X)+xZeroSteps;
		else
			xstep = xCurrentSteps;

		if(y!=DNULL)
			ystep = (y / MM_PER_STEP_Y)+yZeroSteps;
		else
			ystep = yCurrentSteps;

		if(z!=DNULL)
			zstep = (z / MM_PER_STEP_Z)+zZeroSteps;
		else
			zstep = zCurrentSteps;

		SetFeed(XY_FAST_FEED, Z_FAST_FEED);
		gotoStepXYZ(xstep,ystep,zstep);

	}
	else if(block.indexOf("G01")!=-1)
	{
		x=findValue(block,"X");
		y=findValue(block,"Y");
		z=findValue(block,"Z");

		if(x!=DNULL)
			xstep = (x / MM_PER_STEP_X)+xZeroSteps;
		else
			xstep = xCurrentSteps;

		if(y!=DNULL)
			ystep = (y / MM_PER_STEP_Y)+yZeroSteps;
		else
			ystep = yCurrentSteps;

		if(z!=DNULL)
			zstep = (z / MM_PER_STEP_Z)+zZeroSteps;
		else
			zstep = zCurrentSteps;

		SetFeed(feedxy, feedz);
		gotoStepXYZ(xstep,ystep,zstep);
	}
	else if(block.indexOf("G02")!=-1)
	{
		x=findValue(block,"X");
		y=findValue(block,"Y");
		CCI=findValue(block,"I");
		CCJ=findValue(block,"J");

		if(x!=DNULL)
			xstep = (x / MM_PER_STEP_X)+xZeroSteps;
		else
			xstep = xCurrentSteps;

		if(y!=DNULL)
			ystep = (y / MM_PER_STEP_Y)+yZeroSteps;
		else
			ystep = yCurrentSteps;

		if(CCI==DNULL || CCJ==DNULL)
			Alarm("BAD ARC");

		SetFeed(feedxy, feedz);
		goClockwise(xstep,ystep,CCI/MM_PER_STEP_X,CCJ/MM_PER_STEP_Y);  
	}
	else if(block.indexOf("G03")!=-1)
	{
		x=findValue(block,"X");
		y=findValue(block,"Y");
		CCI=findValue(block,"I");
		CCJ=findValue(block,"J");

		if(x!=DNULL)
			xstep = (x / MM_PER_STEP_X)+xZeroSteps;
		else
			xstep = xCurrentSteps;

		if(y!=DNULL)
			ystep = (y / MM_PER_STEP_Y)+yZeroSteps;
		else
			ystep = yCurrentSteps;

		if(CCI==DNULL || CCJ==DNULL)
			Alarm("BAD ARC");

		SetFeed(feedxy, feedz);
		goAntiClockwise(xstep,ystep,CCI/MM_PER_STEP_X,CCJ/MM_PER_STEP_Y);  
	}
	else if(block.indexOf("G04")!=-1)	// Format G04 P1000
	{
		double p = findValue(block,"P");
		if(p!=DNULL)
			delay((unsigned long)p);
	}
	else if(block.indexOf("M06")!=-1)	// Format T# M06
	{
		int t =(int)findValue(block,"T");
		SetFeed(XY_FAST_FEED, Z_FAST_FEED);
		gotoStepZ(TOOLCHANGEPOS_Z);
		gotoStepXYZ(TOOLCHANGEPOS_X, TOOLCHANGEPOS_Y, TOOLCHANGEPOS_Z);
		digitalWrite(SPINDLEPIN,0);
		paused = true;
		toolChange = true;
		changeToTool = t;
	}
	else if(block.indexOf("G28")!=-1)
	{
		SetFeed(XY_FAST_FEED, Z_FAST_FEED);
		gotoStepZ(ZSTEPS_MAX);
		gotoStepXYZ(0,YSTEPS_MAX,ZSTEPS_MAX);
	}
	else if(block.indexOf("G73")!=-1)
	{
		double totDepth = findValue(block,"Z");
		double incDepth = findValue(block,"Q");

		if(totDepth == DNULL || incDepth == DNULL)
			Alarm("G73 ERROR");


		int totDepthStep = totDepth /MM_PER_STEP_Z + zZeroSteps;
		int incDepthStep = incDepth /MM_PER_STEP_Z;
		int retrectSteps = incDepthStep /2;
		int startDepthSteps = zCurrentSteps;

		while(zCurrentSteps > totDepthStep)
		{
			if(zCurrentSteps- incDepthStep > totDepthStep)
			{
				// Feed down
				SetFeed(feedxy, feedz);
				gotoStepZ(zCurrentSteps - incDepthStep);
				// Retract a bit
				SetFeed(feedxy, Z_FAST_FEED);
				gotoStepZ(zCurrentSteps + retrectSteps);
				// Feed back down
				SetFeed(feedxy, feedz);
				gotoStepZ(zCurrentSteps - incDepthStep);
			}
			else
			{
				// Feed down
				SetFeed(feedxy, feedz);
				gotoStepZ(totDepthStep);
				break;
			}
		}

		// Retract completely
		SetFeed(feedxy, Z_FAST_FEED);
		gotoStepZ(startDepthSteps);
	}

	PrintAxis();
}

double findValue(String block, String v)
{
	int pos = block.indexOf(v);
	if(pos==-1)
		return DNULL;

	char line[64];
	block.substring(pos+1).toCharArray(line,64);

	return atof(line);
}

void doJogging()
{
	jogging = true;
	while(jogging)
	{
		xjoy=analogRead(JOY_X);
		yjoy=analogRead(JOY_Y);

		jogging = false;
		if(xjoy< 400)
		{
			if(xCurrentSteps<=0)
				return;       	
			stepaxis(&xstepper,-XJOG_STEPS);
			xCurrentSteps-=XJOG_STEPS;
			lastJogAxis = 1;
			jogging = true;
		}
		else if(xjoy> 600)
		{  
			if(xCurrentSteps>=XSTEPS_MAX)
				return;  
			stepaxis(&xstepper, XJOG_STEPS);
			xCurrentSteps+=XJOG_STEPS;
			lastJogAxis = 1;
			jogging = true;
		}

		if(yjoy< 400)
		{
			if(XZ_jog)
			{
				if(zCurrentSteps<= 0)
					return;   

				zstepper.step(-ZJOG_STEPS);
				zCurrentSteps-=ZJOG_STEPS;
				lastJogAxis = 3;
			}
			else
			{
				if(yCurrentSteps<= 0)
					return;    

				stepaxis(&ystepper, -YJOG_STEPS);
				yCurrentSteps-=YJOG_STEPS;
				lastJogAxis = 2;
			}
			jogging = true;
		}
		else if(yjoy> 600)
		{
			if(XZ_jog)
			{
				if(zCurrentSteps>= ZSTEPS_MAX)
					return;

				stepaxis(&zstepper, ZJOG_STEPS);
				zCurrentSteps+=ZJOG_STEPS;
				lastJogAxis = 3;
			}
			else
			{
				if(yCurrentSteps>=YSTEPS_MAX)
					return;

				stepaxis(&ystepper, YJOG_STEPS);
				yCurrentSteps+=YJOG_STEPS;
				lastJogAxis = 2;
			}
			jogging = true;
		}
	}
}

void stepaxis(Stepper *stepper, int steps)
{
	// F and FZ is in mm/min
	int cfxy, cfz;

	for(int s = 0; s< abs(steps); s++)
	{
		feedoverride = map(analogRead(POT), 1024, 0, 5,20)/10.0;
		cfxy = pfxy * feedoverride;
		cfz = pfz * feedoverride;

		if(cfxy > XY_FAST_FEED)
			cfxy = XY_FAST_FEED;
		if(cfz > Z_MAX_FEED)
			cfz = Z_MAX_FEED;
		xstepper.setSpeed(cfxy);
		ystepper.setSpeed(cfxy);
		zstepper.setSpeed(cfz);

		stepper->step(sgn(steps));
		checkLimitsAndStops();
	}
}

static inline int8_t sgn(int val) 
{
	if (val < 0) return -1;
	if (val==0) return 0;
	return 1;
}