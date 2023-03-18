 //TODO convert verbose and protect into varables, for UI control
//#define VERBOSE  //talk about stuff
//#define LIGHT_VERBOSE  //Most recent light reading
//#define SONAR_VERBOSE //see each sonar reading
//#define SCAN_VERBOSE //raw sonar scan data
//#define PWR_VERBOSE //see each sonar reading
//#define BATT_VERBOSE //see each Battery reading
//#define LCD_VERBOSE //raw sonar scan data
//#define LED_VERBOSE //raw LED activity
//#define XY_VERBOSE //raw LED activity
#define RENDER_VERBOSE //raw LED activity
//#define CFG_VERBOSE // config updates
//#define BATT_VERBOSE // config updates

//#define PROTECT  //try to turn off the battery if UVLO event detected
//#define SIMULATE //Make fake values for not being plugged in 

#include <Wire.h>
#include <BH1750.h>
#include <DS3231.h>
//#define SSD1306_128_32
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>
#include <FastLED.h>

//***************** LED Defines **************************************************
// This assumes the LED display sequence width-wise first;
// Index 0 in corner, 1 is right or left of that

//#define DEV1 //uses a 16x16 display
#define DEV2 //uses a 8x32 display
//#define PROD //PortoDisco, uses Qty6 7x7 nets, +1 between.

#ifdef DEV1
  #define BRIGHTNESS  64  // TODO make this a variable
  #define COLOR_ORDER GRB
  #define CHIPSET     WS2812
  #define PANELS 1 // How many panels per strand, assumes identical
  #define kMatrixWidth 16 // How wide is the panel
  #define kMatrixHeight 16 // How tall is the panel
  #define ADDITIONAL 0 // any extra LEDs after a panel before the next
  #define STRANDS 1 // FastLED outputs in use
#endif

#ifdef DEV2
  #define BRIGHTNESS  64  // TODO make this a variable
  #define COLOR_ORDER GRB
  #define CHIPSET     WS2812
  #define PANELS 1 // How many panels per strand, assumes identical
  #define kMatrixWidth 8 // How wide is the panel
  #define kMatrixHeight 32 // How tall is the panel
  #define ADDITIONAL 0 // any extra LEDs after a panel before the next
  #define STRANDS 2 // FastLED outputs in use
#endif

#ifdef PROD
  #define BRIGHTNESS  128  // TODO make this a variable
  #define COLOR_ORDER RGB
  #define CHIPSET     WS2811_400
  #define PANELS 3 // How many panels per strand, assumes identical
  #define kMatrixWidth 7 // How wide is the panel
  #define kMatrixHeight 7 // How tall is the panel
  #define ADDITIONAL 1 // any extra LEDs after a panel before the next
  #define STRANDS 2 // FastLED outputs in use
#endif

#define NUM_LEDS ( ( (kMatrixWidth * kMatrixHeight) + ADDITIONAL) * PANELS)
#define MAX_PANEL (STRANDS * PANELS) //total number of LEDs panels
#define PANEL_SIZE (kMatrixWidth * kMatrixHeight) //the number of active LEDs used in a panel
#define SEG_SIZE (PANEL_SIZE + ADDITIONAL) //total LED count per panel

//Based on calculated values
// full scale Vbat_fs = 29.56V 
// R1 = 37.4K   R2 = 4.7K   Vref = 3.3V   Abit = 1024
// 3.3 * (8.957) / 1024 = 28.867mV/div
//#define SCALE ((3.3 * 8.957) / 1024)
#define SCALE 0.027126  //from measurements

//Measured PCB behavior
//9.69V = 361#
//12.61V = 470#
//16.36V = 607#
// from excel trendline:
// Count = 36.865 * Volts + 4.2612
// volts = 0.027126 * count - 0.1155

#define MIN_CELL_V 3.2
//minimum A2D val/voltage based on number of cells
// cells, min volts, A2D count
//1  3.2   122.228
//2  6.4   240.196
//3  9.6   358.164
//4  12.8  476.132
//5  16    594.1
//6  19.2  712.068

//*****************Hardware Defines **************************************************
// Direct Hardware wiring
#define LED_A 12 //LED_output A
#define LED_B 13 //LED output B
#define SDA 2 // SDA
#define SCK A5 // SCK
#define OLED_RESET 16
#define VBATT A0 //Voltage divided battery. 
// full scale Vbat_fs = 29.56V 
// R1 = 37.4K   R2 = 4.7K   Vref = 3.3V   Abit = 1024
// 3.3 * (8.957) / 1024 = 28.867mV/div

// Input MCP23008 I/O expander
#define INadr 0 // Address of the MCP323008 device
#define ECHO_A1 0 //Sonar input 
#define ECHO_A2 1 //Sonar input 
#define ECHO_A3 2 //Sonar input 
#define ECHO_B1 3 //Sonar input 
#define ECHO_B2 4 //Sonar input 
#define ECHO_B3 5 //Sonar input 
#define VOUTA 6 // LED_A output Voltage after poly fuse
#define VOUTB 7 // LED_B output Voltage after poly fuse

// Output MCP23008 I/O expander
#define OUTadr 1 // Address of the MCP323008 device
#define DC_EN_A 1 //Enable pin for DC-DC of CH A
#define DC_EN_B 0 //Enable pin for DC-DC of CH A
#define TRIGGER 2 //Sonar ping trigger
#define SENSOR_PWR  3 //multi use
#define RUN 4 //Low Voltage disconnect EN (HIGH holds power on)

// Opperational Constants ****************************************************
#define COLOR_MAX 5 //maximum possible color algorythms
#define EFFECT_MAX 6 //maximum possible brightness effects algorythms
#define AMPL_MIN -100 //effect amplitude minimum
#define AMPL_MAX 100 //effect amplitude maximum
#define SKEW_MAX 255 //possible hue shifts from global count
#define SHIFT_SPEED 0.15 //Base LED animation speed
#define ROLL_SPEED 5  //Base Hue transition rate 
#define ROLL_SCALE 10  //Hue offset per pixel 
#define UVLO_DELAY 5000  //Interval to test battery
#define SENSOR_DELAY 1000  //Interval to power sonar sensors
#define SONAR_DELAY 100  //delay after power up before testing
#define LED_DELAY 41  //interval between LED frame renders
#define LIGHT_DELAY 8000  //interval between Light sensor readings
#define LCD_DELAY 500  //interval between OLED updates
#define PWR_DELAY 1000  //interval between LED power output evaluations 
#define TRIGGER_TIME 20  //Time to run lights after detection
#define FADE_TIME 5  //seconds to fade in/out
#define RETRIGGER_TIME (TRIGGER_TIME-FADE_TIME)  //extensions of run time for redetections
#define CFG_REFRESH 10 //seconds to refresh an actively running channel 
#define SONAR_SIZE 6  //there are 6 channels of sonar
#define METER 5800  //microseconds for sound to go out and back 1 meter
#define SONAR_LOOP 785 // Sonar scan loop rate in microseconds 
#define ACCURACY (METER/SONAR_LOOP) //Samples per meter 5800/785=7.34, inverse 0.135m  
#define RANGE 3 //sonar range in meters
#define SCAN_SIZE (RANGE*ACCURACY)
#define TRIGGER_DISTANCE 1.2  //distance to trigger dance parrty reaction
#define COURTACY_DISTANCE .3  //distance for all white light

//***************** Cordinate Struct **************************************************
struct Cord{
  byte s; // Strand which this LED belongs to. 
  byte i; // index number in the led data array (0 to infinity)
  byte p; // Panel which this LED belongs to. 
  byte x; // X position in geater array
  byte y; // X position in geater array
  byte dx; // Local X position in array
  byte dy; // Local Y position in array
} ;

//***************** Config Struct **************************************************
struct channel_cfg { 
  bool present;  //sensor detection result 
  byte live;  //activity coutdown timer 
  byte color;  //color render mode
  byte effect;  //brightness render mode
  byte Xampl;  //X render amplitude
  byte Yampl;  //Y render amplitude
  byte skew;  //render color offset from global
  byte fade;  //Inverse brightness Gain (1=bright) 
  byte refresh;  //count down timer till cfg refresh
} ;

//***************** Library Defines **************************************************
DS3231 Clock;
RTCDateTime dt;   //Usage: dt = Clock.getDateTime();
BH1750 lightMeter;
float lux;
Adafruit_SSD1306 lcd(OLED_RESET);
Adafruit_MCP23008 OUTmcp;
Adafruit_MCP23008 INmcp;
//  Global Variable declaration ###############################################################

//  Timers used to sequence various jobs  **************************************************
channel_cfg cfg[SONAR_SIZE];//things like animation modes, fade positions, etc, configuration for each channel

unsigned long sonar_time;  // Delay after sensor power is Enabled
unsigned long uvlo_time;  // Is the battery voltage safe to operate
unsigned long sensor_time;  // Power up sensors in preparation for a sonar scan
unsigned long led_time;  // LED update rate, 41mS for 24Hz, minimum for smooth & minimum demand 
unsigned long light_time;  // Measure ambient light every 60 seconds
unsigned long lcd_time;  // Update the OLED display with new readings
unsigned long PWR_time;  // Update the LED power outputs, saves 0.75W if no outputs are on

// Sonar variables  **************************************************
double sonar[SONAR_SIZE];  //resultant measurements of sonar scans
bool sonar_active = false;  //holds sonar sensor power state

// Battery variables  **************************************************
byte cells = 3; //literal count, not 0 indexed
double volts;  //holds latest voltage reading in volts

// LED variables  **************************************************
byte roll_color;  //global Hue index
float shift;  //global motion index
const float pi = 6.283185307179586476925286766559;

#if STRANDS == 1
  CRGB leds[ NUM_LEDS ]; //Led array A
#endif
#if STRANDS == 2
  CRGB leds[ STRANDS ][ NUM_LEDS ]; //Led array A
#endif

// LED display XY solver **************************************************
// populates all the fields of a coordinate based on an index number
#ifdef DEV1
struct Cord solveXY (int k, byte s) {
  Cord result;
  result.i = k;
  result.s = s;
  result.y = k / kMatrixWidth;
  result.x = k - (kMatrixWidth * result.y); //we will always offset by complete rows
  if ( result.y % 2 ) { //solve odd row
    result.x = (kMatrixWidth-1) - result.x;
  }
  // panel specific configuration 16x16
  if (result.x <= 6 && result.y <= 6) { //panel 1
    result.p = 0;
    result.dy = result.y;
    result.dx = result.x;
  } else if (result.x >= 9 && result.y <= 6) {//panel 2
    result.p = 1;
    result.dy = result.y;
    result.dx = result.x - 9;
  } else if (result.x <= 6 && result.y >= 9) {//panel 3
    result.p = 2;
    result.dy = result.y - 9;
    result.dx = result.x;
  } else if (result.x >= 9 && result.y >= 9) {//panel 4
    result.p = SONAR_SIZE+1;
    result.dy = result.y - 9;
    result.dx = result.x - 9;
  } else {
    result.p = SONAR_SIZE+1;
    result.dy = 0;
    result.dx = 0;
  }
  if (s) {
    result.p += 3;
  }
  cfg[loc.p].fade = step2fade(cfg[result.p].live); //solve any fade 
  return result;
}
#endif

#ifdef DEV2
struct Cord solveXY (int k, byte s) {
  Cord result;
  result.i = k;
  result.s = s;
  result.y = k / kMatrixWidth;
  result.x = k - (kMatrixWidth * result.y); //we will always offset by complete rows
  if ( result.y % 2 ) { //solve odd row
    result.x = (kMatrixWidth-1) - result.x;
  }
  // panel specific configuration 32x8
  if (result.x <= 6 && result.y <= 6) { //panel 1
    result.p = 0;
    result.dy = result.y;
    result.dx = result.x;
  } else if (result.x <= 6 && result.y >= 8 && result.y <= 14) {//panel 2
    result.p = 1;
    result.dy = result.y - 8;
    result.dx = result.x;
  } else if (result.x <= 6 && result.y >= 16 && result.y <= 22) {//panel 3
    result.p = 2;
    result.dy = result.y - 16;
    result.dx = result.x;
  } else if (result.x <= 6 && result.y >= 24 && result.y <= 30) {//panel 4
    result.p = SONAR_SIZE+1; //too many screes available
    result.dy = result.y - 24;
    result.dx = result.x;
  } else {
    result.p = SONAR_SIZE+1;
    result.dy = 0;
    result.dx = 0;
  }
  if (s) {
    result.p += 3;
  }
  if (result.p < SONAR_SIZE) {
    cfg[result.p].fade = step2fade(cfg[result.p].live); //solve any fade 
  }
  return result;
}
#endif

#ifdef PROD
struct Cord solveXY (int k, byte s) {
  Cord result;
  result.i = k; //save the index of the current LED
  result.s = s; //save which strand it is on
  result.p = (k / SEG_SIZE);  //solve which sequential panel is being used
  k = result.i - (result.p * SEG_SIZE); // solve local panel index
  result.p = (PANELS * s) + result.p;  //solve acual panel
  if (k >= PANEL_SIZE) { //check if out of panel scope
    result.dy = result.y = result.x = result.dx = 255;
    result.p = SONAR_SIZE+1;
  } else {
    result.dy = k / kMatrixWidth;
    result.y = result.dy + (kMatrixHeight * result.p);
    result.x = k - (kMatrixWidth * result.dy); //we will always offset by complete rows
    if ( result.dy % 2 ) { //solve odd row
      result.x = (kMatrixWidth-1) - result.x;
    }
    result.dx = result.x; 
  }
  if (result.p < SONAR_SIZE) {
    cfg[result.p].fade = step2fade(cfg[result.p].live); //solve any fade 
  }
  return result;
}
#endif

//fade (inverse) gain based on steps from start/end
byte step2fade(byte timer) {
  switch (timer) {
    case TRIGGER_TIME:
      return 5;
    case TRIGGER_TIME-1:
      return 4;
    case TRIGGER_TIME-2:
      return 3;
    case TRIGGER_TIME-3:
      return 2;
    case TRIGGER_TIME-4:
      return 1;
    default:
      return 0;
    case 5:
      return 1;
    case 4:
      return 2;
    case 3:
      return 3;
    case 2:
      return 4;
    case 1:
      return 5;
    case 0:
      return 8;
  }
}


// brightness/intensity filters **********************
byte Xripple (Cord loc) {
  return 255 * sin((shift + cfg[loc.p].Xampl * loc.dx)/8);
}

byte Yripple (Cord loc) {
  return 255 * sin((shift + cfg[loc.p].Yampl * loc.dy)/8);
}

byte ripple (Cord loc) {
  return Xripple(loc)+Yripple(loc);
}

unsigned long render_time = micros();

CRGB color(Cord loc) {
  //update transition seeds at beginning of frame
  if (loc.x == 0 && loc.y == 0 && loc.s == 0) {
    roll_color += ROLL_SPEED;
    shift -= SHIFT_SPEED;
  } 

  //if not on an actual panel, if not active or          if not a present LED, 
  if (loc.p >= SONAR_SIZE || cfg[loc.p].live == false || cfg[loc.p].present == false) { 
    #ifdef RENDER_VERBOSE
      Serial.print("Z");
      if (loc.p >= SONAR_SIZE) {Serial.print("s");}
      if (cfg[loc.p].live == false) {Serial.print("l");}
      if (cfg[loc.p].present == false) {Serial.print("p");}
    #endif  
    return CHSV(0, 255, 0); // return dark
  } 

  byte Hue = roll_color + cfg[loc.p].skew;  //preload with color information
  //color filters
  switch (cfg[loc.p].color) {
  case 0:
  default://  solid
    #ifdef RENDER_VERBOSE
      Serial.print("Sl,");
    #endif  
    Hue +=  loc.i;
    break;
  case 1: //rolling rainbow
    #ifdef RENDER_VERBOSE
      Serial.print("Rl");
    #endif  
      Hue += ((((unsigned int)cfg[loc.p].Xampl>>3)/*divide by 8*/ * loc.dx)<<3) + ((((unsigned int)cfg[loc.p].Yampl>>3) * loc.dy)<<3);
    break;
  case 2: //  rollCircle
    #ifdef RENDER_VERBOSE
      Serial.print("Cr");
    #endif  
    Hue += ((((unsigned int)cfg[loc.p].Xampl>>3) * abs(3-loc.dx))<<3) + ((((unsigned int)cfg[loc.p].Yampl>>3) * abs(3-loc.dy))<<3);
    break;
  case 3:  // checkersColor;
    #ifdef RENDER_VERBOSE
      Serial.print("Cr");
    #endif  
    if ( (loc.dx + loc.dy + (byte)shift) % 2 ) {
      Hue += 80;
    }
    break;
  case 4: // superCheckersColor;
    #ifdef RENDER_VERBOSE
      Serial.print("Cr");
    #endif  
    byte sw = (loc.dx - loc.dy + (byte)shift) % 3 ;
    switch (sw)  {
      default:
      case 0:
        break;
      case 1:
        Hue += 85;
        break;
      case 2:
        Hue += 170;
        break;
    }
    break;
  }
  

  int Value = 255; 
  //brightness filter
  switch (cfg[loc.p].effect) {
  case 0:
  default:  // full   Value = 255;  set as default when initialised
    #ifdef RENDER_VERBOSE
      Serial.print("Fl");
    #endif  
    break;
  case 1: //rolling rainbow
    #ifdef RENDER_VERBOSE
      Serial.print("Rx");
    #endif  
    Value = Xripple(loc);
    break;
  case 2: //rolling rainbow
    #ifdef RENDER_VERBOSE
      Serial.print("Ry");
    #endif  
    Value = Yripple(loc);
    break;
  case 3: //rolling rainbow
    #ifdef RENDER_VERBOSE
      Serial.print("Rp");
    #endif  
    Value = Xripple(loc)+Yripple(loc);
    break;
  case 4: //    Value = checkers(loc);
    #ifdef RENDER_VERBOSE
      Serial.print("Ch");
    #endif  
    if ( (loc.dx + loc.dy + (byte)shift) % 2 ) {
      Value = 127;
    }
    break;
  case 5:
    #ifdef RENDER_VERBOSE
      Serial.print("CH");
    #endif  
//    Value = superCheckers(loc);
    byte sw = (loc.dx - loc.dy + (byte)shift) % 3 ;
    switch (sw)  {
      default:
      case 0:
        break;
      case 1:
        Value = 85;
        break;
      case 2:
        Value = 170;
        break;
    }
    break;
  }

  Value = (Value >> cfg[loc.p].fade);
//  Value = cfg[loc.p].fade * Value / 255; //apply fade inverse gain

  //limit adjustments values
  if (Hue > 255) {Hue = 255;}
  if (Hue < 0) {Hue = 0;}
  if (Value > 255) {Value = 255;}
  if (Value < 0) {Value = 0;}
  
    #ifdef LED_VERBOSE
      Serial.print('H');
      Serial.print(Hue);
      Serial.print('V');
      Serial.print(Value);
      Serial.print("|");
    #endif  
  return CHSV((byte)Hue, 255, (byte)Value);
}

void renderFrame() {
  for (int s = 0; s < STRANDS; s++) {  //go through strands
      #ifdef LED_VERBOSE
        Serial.print("\nS");
        Serial.print(s);
      #endif  
    for (int l = 0; l < NUM_LEDS; l++) {  //go through leds on strand
      #ifdef LED_VERBOSE
        Serial.print("_");
        Serial.print(l);
        Serial.print("(");
      #endif  
      #if STRANDS == 1
        leds[l] = color(solveXY(l,s));  //solve the color of each LED
      #endif
      #if STRANDS == 2
        Cord result = solveXY(l,s);
        #ifdef XY_VERBOSE
          Serial.print(" Col s");
          Serial.print(result.s);
          Serial.print("i");
          Serial.print(result.i);
          Serial.print("p");
          Serial.print(result.p);
          Serial.print("x");
          Serial.print(result.x);
          Serial.print("y");
          Serial.print(result.y);
          Serial.print("X");
          Serial.print(result.dx);
          Serial.print("Y");
          Serial.print(result.dy);
        #endif  
        leds[s][l] = color(result);  //solve the color of each LED
      #endif
      #ifdef LED_VERBOSE
        Serial.print(leds[s][l].r);
        Serial.print(",");
        Serial.print(leds[s][l].g);
        Serial.print(",");
        Serial.print(leds[s][l].b);
        Serial.print(")");
        delay(50);
      #endif  
    }
  }
}

void updateLED() { //render first leads to more consistent frame rate
  if (millis() < led_time) {return;}
  led_time = millis() + LED_DELAY;

  #ifdef LED_VERBOSE
    Serial.print("->");
  #endif  
  FastLED.show(); //output last rendered frame

  #ifdef RENDER_VERBOSE
  render_time = micros();
  #endif  
  
  renderFrame();
  #ifdef RENDER_VERBOSE
  Serial.println(micros() - render_time);
  #endif  
  #ifdef LED_VERBOSE
    Serial.print("#");
  #endif  
  
}

//***************** Configuration Updates **************************************************
void tellCfg(byte ch){
  #ifdef VERBOSE
    Serial.print("/nCfg");
  #endif
  #ifdef CFG_VERBOSE
    Serial.print("#:");
    Serial.print(ch);
    Serial.print(" c:");
    Serial.print(cfg[ch].present);
    Serial.print(" e:");
    Serial.print(cfg[ch].live);
    Serial.print(" c:");
    Serial.print(cfg[ch].color);
    Serial.print(" e:");
    Serial.print(cfg[ch].effect);
    Serial.print(" x:");
    Serial.print(cfg[ch].Xampl);
    Serial.print(" y:");
    Serial.print(cfg[ch].Yampl);
    Serial.print(" s:");
    Serial.print(cfg[ch].skew);
    Serial.print(" f:");
    Serial.println(cfg[ch].fade);
  #endif
}

void updateChannelCfg(byte ch) {
  #ifdef SIMULATE
    cfg[ch].color  = 0;
    cfg[ch].effect = 0;
    cfg[ch].Xampl  = 1; //because negative min, random limited to span, then offset
    cfg[ch].Yampl  = 1;
    cfg[ch].skew   = 0;
    cfg[ch].fade   = 255; //reset the fade
  #endif
    
  #ifndef SIMULATE
    cfg[ch].color  = rand()%COLOR_MAX;  //bounded random number 
    cfg[ch].effect = rand()%EFFECT_MAX;
    cfg[ch].Xampl  = ( rand()%(AMPL_MAX-AMPL_MIN) )+AMPL_MIN; //because negative min, random limited to span, then offset
    cfg[ch].Yampl  = ( rand()%(AMPL_MAX-AMPL_MIN) )+AMPL_MIN;
    cfg[ch].skew   = rand()%SKEW_MAX;
    cfg[ch].fade   = 0; //reset the fade
    cfg[ch].refresh   = CFG_REFRESH; //refresh settings configuration
  #endif
  
  tellCfg(ch);//
}

//***************** Light Sensor **************************************************
void readLight() {
  if (millis() < light_time) {return;} //not time yet
  light_time = millis() + LIGHT_DELAY;
  
  lux = lightMeter.readLightLevel(); //returns int of lux reading
  
  #ifdef VERBOSE
    Serial.print("\nLight");
  #endif
  #ifdef LIGHT_VERBOSE
    Serial.print("Reading: ");
    Serial.print(lux);
    Serial.print("lux");
  #endif
}

//***************** Battery Check Sensor **************************************************
// test for unsafe voltage, shut off power if nessesary
void UVLOcheck() {
  if (millis() < uvlo_time) {return;} //not time yet
  uvlo_time = millis() + UVLO_DELAY;

  #ifdef SIMULATE
    volts = 370*SCALE;
  #endif

  #ifndef SIMULATE
    volts = analogRead(VBATT)*SCALE;
  #endif

  #ifdef VERBOSE
    Serial.print("\nBattery");
  #endif
  #ifdef BATT_VERBOSE
    Serial.print(" Reading: "); Serial.print(volts); Serial.print("V");
  #endif

  #ifdef PROTECT
    if (volts < MIN_CELL_V * cells) {
      
      lcd.clearDisplay();
      lcd.setCursor(0,0);
      lcd.setTextColor(BLACK, WHITE); // 'inverted' text
      lcd.print("Shutting down, Low Battery"); 
      lcd.display();

      Serial.print("\nShutting down, Low Battery");
      delay(50);
      
      OUTmcp.digitalWrite(RUN, LOW); // turn OFF power supply enable
      //Should power down, only start button press would keep awake, hense user message
      while (1) {} //wait for eternity
    }
  #endif
}

//***************** LED Power Service **************************************************
//runs to determine which stalls are occupied or vacant
//occupied get updated accordingly
//vacant timeout and go dark.
//unnessesary power outputs are powered off.
void updatePWR(){
  if (millis() < PWR_time) {return;}
  PWR_time = millis() + PWR_DELAY;
  
  #ifdef VERBOSE
    Serial.print("\nPWR Update (");
  #endif
  //count down non-zero timers
  for (int i = 0; i < SONAR_SIZE; i++) {
    if (cfg[i].live > 0) {
      cfg[i].live -= 1; //count down 1 second
      cfg[i].refresh -= 1; //count down 1 second
      if (cfg[i].refresh == 0) {
        cfg[i].refresh = CFG_REFRESH;
        updateChannelCfg(i);  //randomly get a new configuration
      }
      #ifdef PWR_VERBOSE
        Serial.print(i);
        Serial.print(':');
        Serial.print(cfg[i].live);
        if (i<SONAR_SIZE-1) {Serial.print(", ");}
      #endif
    }
  }
  #ifdef PWR_VERBOSE
    Serial.print(')');
  #endif
  
  //turn ON/OFF LED power supplies as nessesary
  if ( (cfg[3].live * cfg[3].present) || (cfg[4].live * cfg[4].present) || (cfg[5].live * cfg[5].present) ) {
    OUTmcp.digitalWrite(DC_EN_A, HIGH); // turn on LED_A power supply enable
    #ifdef PWR_VERBOSE
      Serial.print("A ON, ");
    #endif
  } else {
    OUTmcp.digitalWrite(DC_EN_A, LOW); // turn off LED_A power supply enable
    #ifdef PWR_VERBOSE
      Serial.print("A OFF, ");
    #endif
  }
  if ( (cfg[0].live * cfg[0].present) || (cfg[1].live * cfg[1].present) || (cfg[2].live * cfg[2].present) ) {
    OUTmcp.digitalWrite(DC_EN_B, HIGH); // turn on LED_A power supply enable
    #ifdef PWR_VERBOSE
      Serial.print("B ON.");
    #endif
  } else {
    OUTmcp.digitalWrite(DC_EN_B, LOW); // turn off LED_A power supply enable
    #ifdef PWR_VERBOSE
      Serial.print("B OFF.");
    #endif
  }
  
}

void chgT(byte inv) {
  if (inv > 0) {
    lcd.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    lcd.setTextColor(WHITE);
  }
}

//***************** OLED Service **************************************************
void updateLcd () {
  if (millis() < lcd_time) {return;} //not time yet
  lcd_time = millis() + LCD_DELAY;

  #ifdef VERBOSE
    Serial.print("\nLCD Update ");
  #endif
    
  lcd.clearDisplay();
  
  lcd.setCursor(0,0);
  chgT(cfg[0].present);
  lcd.print("1B"); 
  chgT(cfg[0].live);
  lcd.print(sonar[0]);
  
  lcd.setCursor(0,8);
  chgT(cfg[1].present);
  lcd.print("2B"); 
  chgT(cfg[1].live);
  lcd.print(sonar[1]);
  
  lcd.setCursor(0,16);
  chgT(cfg[2].present);
  lcd.print("3B"); 
  chgT(cfg[2].live);
  lcd.print(sonar[2]);
  
  lcd.setCursor(92,0);
  chgT(cfg[3].present);
  lcd.print("1A"); 
  chgT(cfg[3].live);
  lcd.print(sonar[3]);
  
  lcd.setCursor(92,8);
  chgT(cfg[4].present);
  lcd.print("2A"); 
  chgT(cfg[4].live);
  lcd.print(sonar[4]);
  
  lcd.setCursor(92,16);
  chgT(cfg[5].present);
  lcd.print("3A"); 
  chgT(cfg[5].live);
  lcd.print(sonar[5]);

  #ifdef LCD_VERBOSE
    Serial.print('*');
  #endif
   
  chgT(0); //Back to normal
  lcd.setCursor(16,24);
  dt = Clock.getDateTime();
  #ifdef LCD_VERBOSE
    Serial.print('!');
  #endif
   
  lcd.print(dt.year);   lcd.print("-");
  lcd.print(dt.month);  lcd.print("-");
  lcd.print(dt.day);    lcd.print(" ");
  lcd.print(dt.hour);   lcd.print(":");
  lcd.print(dt.minute); lcd.print(":");
  lcd.print(dt.second); 
  
  #ifdef LCD_VERBOSE
    Serial.print('^');
  #endif

  lcd.setCursor(48,8);
//  lcd.print("Battery:");
  lcd.print(volts);  chgT(1); lcd.print("V");  chgT(0);
  
  #ifdef LCD_VERBOSE
    Serial.print('%');
  #endif

  lcd.setCursor(48,16);
//  lcd.print("Light: ");  
  lcd.print((int)lux); chgT(1); lcd.print("lx");  chgT(0);
  
  #ifdef LCD_VERBOSE
    Serial.print('~');
  #endif

  lcd.display();

  #ifdef LCD_VERBOSE
    Serial.print('#');
  #endif
}

//***************** Sonar Power Service **************************************************
void sensorPrep () {
  if (millis() < sensor_time) {return;} //not time yet
  
  sensor_time = millis() + SENSOR_DELAY;  // set new alarm for the next sensor sweep 
  sonar_time = millis() + SONAR_DELAY;  // update sonar time to trigger a reading

  #ifdef SONAR_VERBOSE
    Serial.print("\nPWR up sensors");
  #endif

  OUTmcp.digitalWrite(SENSOR_PWR, LOW); //turn ON sensor power, active low P-mos driver
  sonar_active = true;  //mark that the sensors are powered
}

//***************** Sonar Measurement Routine **************************************************
void sonarScan() {
  if (sonar_active == false) {  //sensors are unpowered, no scan possible
    return;  //reason: scan has run, but doesn't have a new time yet
  } 
  
  // sensors powered if here
  if (millis() < sonar_time) { return; } //test if warm up timer expired 
    
  #ifdef SIMULATE
    sonar[0] = 2.3;
    sonar[1] = 1.75;
    sonar[2] = 2.5;
    sonar[3] = 1.02;
    sonar[4] = 1.234;
    sonar[5] = 0.4;
  #endif

  #ifndef SIMULATE  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    for( byte i = 0; i < SONAR_SIZE; i++) {
      sonar[i] = 0;  //reset sonar measurements
    }
    byte scan[SCAN_SIZE];  //make scan memory array
    
    #ifdef SONAR_VERBOSE
      Serial.print("\nChirp Present:");
    #endif

    OUTmcp.digitalWrite(TRIGGER, HIGH); //send chirp pulse
    delayMicroseconds(50);
    OUTmcp.digitalWrite(TRIGGER, LOW); //clear chirp pulse

    unsigned long s_time = micros()+SONAR_LOOP;  //get starting time

    delayMicroseconds(250);
    scan[0] =   INmcp.readGPIO(); //echo will be high if a sensor is attached
    #ifdef SONAR_VERBOSE
      Serial.print(scan[0], 2);
    #endif
    for (byte i = 0; i < SONAR_SIZE; i++) {
      if ( scan[0] & (1<<i) ) { //if the bit is set the sensor is there
        cfg[i].present = true; 
      } else { //bit not set, the sensor is not there
        cfg[i].present = false;
      }
    }
    
    for (byte i = 0; i < SCAN_SIZE; i++) { //delay sample to array
//      scan[i] = (INmcp.digitalRead(ECHO_A1) << ECHO_A1) | (INmcp.digitalRead(ECHO_A2) << ECHO_A2) | (INmcp.digitalRead(ECHO_A3) << ECHO_A3) | (INmcp.digitalRead(ECHO_B1) << ECHO_B1) | (INmcp.digitalRead(ECHO_B2) << ECHO_B2) | (INmcp.digitalRead(ECHO_B3) << ECHO_B3);    

      while (micros() < s_time) {}     //wait for time to pass
      s_time = micros()+SONAR_LOOP;  //set end time for next loop
      scan[i] =   INmcp.readGPIO();  //take reading of port
    }
    
    #ifdef SCAN_VERBOSE
      Serial.print("\nScan: ");
      for (byte i = 0; i < SCAN_SIZE; i++) {  //moving through the scan array
        Serial.print(i);
        Serial.print(':');
        Serial.print(scan[i], 2);
        Serial.print(", ");
      }
      Serial.print("\n");
    #endif
    
    for (byte j = 0; j < SONAR_SIZE; j++) {  //analyse each sonar, pin stays high with no detect
      #ifdef SONAR_VERBOSE
        Serial.print(" Ch");
        Serial.print(j);
        Serial.print(" ");
      #endif
      for (byte i = 0; i < SCAN_SIZE; i++) {  //moving through the scan array
        if ( (1 << j) & scan[i] ) { //if the bit is set save the number
          sonar[j] = i+1; //leaves highest numbered scan result in sonar[]
        }
      }
      #ifdef SONAR_VERBOSE
        Serial.print(sonar[j]);
        Serial.print(", ");
      #endif
      sonar[j] = sonar[j] / ACCURACY; //adjust to actual meter measurement
      #ifdef SONAR_VERBOSE
        Serial.print(sonar[j]);
      #endif
    }
    
  #endif /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

  #ifdef SONAR_VERBOSE
    Serial.println(" Sensor PWR dn");
  #endif
  OUTmcp.digitalWrite(SENSOR_PWR, HIGH); //turn OFF sensor power
  sonar_active = false; //remember sensor power is off
  
  //look through sonar data and extend live timers
  for (int i = 0; i < SONAR_SIZE; i++) { //move through the sonar data
    if (cfg[i].present) { //The sensor was detected after trigger
      if (sonar[i] < TRIGGER_DISTANCE ) {  //if something has been detected
        #ifdef SONAR_VERBOSE
          Serial.print("Det");
          Serial.print(i);
          Serial.print(":");
          Serial.print(sonar[i]);
          Serial.print(",");
          Serial.print(cfg[i].live);
        #endif
        if (cfg[i].live < RETRIGGER_TIME && cfg[i].live > 0) { //check if it was ON but after fade
          cfg[i].live= RETRIGGER_TIME; //save a retrigger
          #ifdef SONAR_VERBOSE
            Serial.print(" RT, ");
          #endif
        } else if (cfg[i].live == 0) { //check if it was off before
          cfg[i].live = TRIGGER_TIME;  //set a countdown clock
          #ifdef SONAR_VERBOSE
            Serial.print(" T, ");
          #endif
          updateChannelCfg(i);  //randomly get a new configuration
        } //else counting down through fade-up period, do nothing
      } else {
        #ifdef SONAR_VERBOSE
          Serial.print(" Clr");
          Serial.print(i);
          Serial.print(",");
          Serial.print(cfg[i].live);
        #endif
      }
    } else { //not present
      #ifdef SONAR_VERBOSE
        Serial.print(" NP");
        Serial.print(i);
      #endif
      cfg[i].live = 0;
    }
  }
}
  
//***************** Main Setup **************************************************
void updateComms () {
    while(Serial.available()) {char c = Serial.read();} //purge input serial buffer to avoid buffer overflows
    //TODO: make recievers to switch battery cell count and battery protection enable
}

//***************** Main Setup **************************************************
void setup() {
  Wire.begin(2,14);
  Serial.begin(115200);  //create seria00000000000000000000l path to debug
//  Serial.begin(1000000);  //create serial path to debug
  #ifdef VERBOSE
    Serial.print("\nBooting");
  #endif
  
  // initialize OLED with the I2C addr 0x3C (for the 128x32)
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  lcd.setTextSize(2);
  lcd.setTextColor(WHITE);
  lcd.print("Porto Party!");
  lcd.display();
  lcd.setTextSize(1);
  
  // Setup direct pin directions 
  pinMode(LED_A, OUTPUT); // LED set A data 
  pinMode(LED_B, OUTPUT); // LED set B data 
  /* handled by oled*/ //  digitalWrite(OLED_RST, HIGH); // take MCP's out of reset
  pinMode(VBATT, INPUT); // Battery Voltage measurement pin
  
  // Setup INmcp pin directions
  INmcp.begin(INadr); //start the IO expander
  INmcp.pinMode(ECHO_A1, INPUT); //sonar echo input
  INmcp.pinMode(ECHO_A2, INPUT);
  INmcp.pinMode(ECHO_A3, INPUT);
  INmcp.pinMode(ECHO_B1, INPUT);
  INmcp.pinMode(ECHO_B2, INPUT);
  INmcp.pinMode(ECHO_B3, INPUT);
  INmcp.pinMode(VOUTA, INPUT);  //LED power output post-polyfuse voltage feedback
  INmcp.pinMode(VOUTB, INPUT);  //Can detect if LEDs are not powered
    
  // Setup OUTmcp pin directions
  OUTmcp.begin(OUTadr); //start the IO expander
  OUTmcp.pinMode(DC_EN_A, OUTPUT);
  OUTmcp.pinMode(DC_EN_B, OUTPUT);
  OUTmcp.pinMode(TRIGGER, OUTPUT);
  OUTmcp.pinMode(SENSOR_PWR, OUTPUT);
  OUTmcp.pinMode(RUN, OUTPUT);
  OUTmcp.digitalWrite(RUN, HIGH); // turn on power supply enable

  UVLOcheck(); // test if safe to operate on battery
  lightMeter.begin();  //start Light device driver

  sensor_time = millis();
  sonar_time = millis() + SONAR_DELAY;
  led_time = millis() + 1000;
  lcd_time = millis() + 300;
  light_time = millis() + 200;
  PWR_time = millis() + 400;

  #ifdef STRANDS == 1
    FastLED.addLeds<CHIPSET, LED_A, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalSMD5050);
  #endif
  #ifdef STRANDS == 2
    FastLED.addLeds<CHIPSET, LED_A, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalSMD5050);
    FastLED.addLeds<CHIPSET, LED_B, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection(TypicalSMD5050);
  #endif
  FastLED.setBrightness( BRIGHTNESS );
  delay (1000);

  for (byte j = 0; j < SONAR_SIZE; j++) {
     updateChannelCfg(j);
  }
  #ifdef VERBOSE
    Serial.print("\nFinished, Running");
  #endif
}

//***************** Main Loop **************************************************
void loop() {
  UVLOcheck();  //test for low bat and shut down if nessesary
  sensorPrep(); //start sonar sensor power 
  sonarScan();  //scan sonar sensors 
  readLight();  //get ambient light senor reading
  updateLcd();  //service the OLED screen
  updatePWR();  //service the LED power outputs
  updateLED();  //render the LED frame 
  updateComms();//service serial connection
}
