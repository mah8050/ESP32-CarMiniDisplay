// An adaption of the "UncannyEyes" sketch (see eye_functions tab)
// for the TFT_eSPI library. As written the sketch is for driving
// one (240x320 minimum) TFT display, showing 2 eyes. See example
// Animated_Eyes_2 for a dual 128x128 TFT display configured sketch.

// The size of the displayed eye is determined by the screen size and
// resolution. The eye image is 128 pixels wide. In humans the palpebral
// fissure (open eye) width is about 30mm so a low resolution, large
// pixel size display works best to show a scale eye image. Note that
// display manufacturers usually quote the diagonal measurement, so a
// 128 x 128 1.7" display or 128 x 160 2" display is about right.

// Configuration settings for the eye, eye style, display count,
// chip selects and x offsets can be defined in the sketch "config.h" tab.

// Performance (frames per second = fps) can be improved by using
// DMA (for SPI displays only) on ESP32 and STM32 processors. Use
// as high a SPI clock rate as is supported by the display. 27MHz
// minimum, some displays can be operated at higher clock rates in
// the range 40-80MHz.

// Single defaultEye performance for different processors
//                                  No DMA   With DMA
// ESP8266 (160MHz CPU) 40MHz SPI   36 fps
// ESP32 27MHz SPI                  53 fps     85 fps
// ESP32 40MHz SPI                  67 fps    102 fps
// ESP32 80MHz SPI                  82 fps    116 fps // Note: Few displays work reliably at 80MHz
// STM32F401 55MHz SPI              44 fps     90 fps
// STM32F446 55MHz SPI              83 fps    155 fps
// STM32F767 55MHz SPI             136 fps    197 fps

// DMA can be used with RP2040, STM32 and ESP32 processors when the interface
// is SPI, uncomment the next line:
// #define USE_DMA

// Load TFT driver library
#include <SPI.h>
#include <TFT_eSPI.h>

//load MPU6050 Library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TFT_eWidget.h>               // Widget library
#include "Final_Frontier_28.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13
Adafruit_MPU6050 mpu;
TFT_eSPI    tft = TFT_eSPI();           // A single instance is used for 1 or 2 displays
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer;
GraphWidget gr = GraphWidget(&tft);    // Graph widget gr instance with pointer to tft
TraceWidget tr = TraceWidget(&gr);     // Graph trace tr with pointer to gr
TraceWidget tr1 = TraceWidget(&gr);
TraceWidget tr2 = TraceWidget(&gr);
TFT_eSprite maxAcc = TFT_eSprite(&tft); //Text
TFT_eSprite HallTemp = TFT_eSprite(&tft); //Text
byte displayPage=2; // Indicate which page should be active


//++++++++++++++++++++TFT_EYES++++++++++++++++++++++++++++++++++++++++++

// A pixel buffer is used during eye rendering
#define BUFFER_SIZE 1024 // 128 to 1024 seems optimum

#ifdef USE_DMA
  #define BUFFERS 2      // 2 toggle buffers with DMA
#else
  #define BUFFERS 1      // 1 buffer for no DMA
#endif

uint16_t pbuffer[BUFFERS][BUFFER_SIZE]; // Pixel rendering buffer
bool     dmaBuf   = 0;                  // DMA buffer selection

// This struct is populated in config.h
typedef struct {        // Struct is defined before including config.h --
  int8_t  select;       // pin numbers for each eye's screen select line
  int8_t  wink;         // and wink button (or -1 if none) specified there,
  uint8_t rotation;     // also display rotation and the x offset
  int16_t xposition;    // position of eye on the screen
} eyeInfo_t;

#include "config.h"     // ****** CONFIGURATION IS DONE IN HERE ******

extern void user_setup(void); // Functions in the user*.cpp files
extern void user_loop(void);

#define SCREEN_X_START 0
#define SCREEN_X_END   SCREEN_WIDTH   // Badly named, actually the "eye" width!
#define SCREEN_Y_START 50
#define SCREEN_Y_END   SCREEN_HEIGHT  // Actually "eye" height

// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

struct {                // One-per-eye structure
  int16_t   tft_cs;     // Chip select pin for each display
  eyeBlink  blink;      // Current blink/wink state
  int16_t   xposition;  // x position of eye image
} eye[NUM_EYES];

uint32_t startTime;  // For FPS indicator

//Grid Implement

const float gxLow  = 0.0;
const float gxHigh = 100.0;
const float gyLow  = -512.0;
const float gyHigh = 512.0;
float maxgy = 0.0;
float maxgx = 0.0;
float maxgz = 0.0;

// INITIALIZATION -- runs once at startup ----------------------------------
void setup(void) {
  Serial.begin(9600);
if (displayPage == 1){
  setupSensorsPage();
}
else if(displayPage == 2){
  setupeyePage();
}
  
//Display eyes

}
void setupeyePage(void){
  #if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
  // Enable backlight pin, initially off
  Serial.println("Backlight turned off");
  pinMode(DISPLAY_BACKLIGHT, OUTPUT);
  digitalWrite(DISPLAY_BACKLIGHT, LOW);
#endif

  // User call for additional features
  user_setup();
  // Initialise the eye(s), this will set all chip selects low for the tft.init()
  initEyes();
  // Initialise TFT
  Serial.println("Initialising displays");
  tft.init();

#ifdef USE_DMA
  tft.initDMA();
#endif

  // Raise chip select(s) so that displays can be individually configured
  digitalWrite(eye[0].tft_cs, HIGH);
  if (NUM_EYES > 1) digitalWrite(eye[1].tft_cs, HIGH);

  for (uint8_t e = 0; e < NUM_EYES; e++) {
    digitalWrite(eye[e].tft_cs, LOW);
    tft.setRotation(eyeInfo[e].rotation);
    tft.fillScreen(TFT_BLACK);
    digitalWrite(eye[e].tft_cs, HIGH);
  }

#if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
  Serial.println("Backlight now on!");
  analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_MAX);
#endif

  startTime = millis(); // For frame-rate calculation
}

void setupSensorsPage(void){
  Wire.setPins(15,16);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }else{
    Serial.println("MPU6050 Found!");
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  }
  Serial.println("Starting Sensor Page");
  // Setup Ds18b20 Temp print address
  if (!sensors.getAddress(Thermometer, 0)) Serial.println("Unable to find address for Device 0");
  Serial.println(sensors.requestTemperatures());
  sensors.setResolution(9);
  sensors.setWaitForConversion(false);  // To Make readings faster
  // TFT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  // Graph area is 200 pixels wide, 150 pixels high, dark grey background
  gr.createGraph(240, 200, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is -512 to 512
  gr.setGraphScale(gxLow, gxHigh, gyLow, gyHigh);
  // X grid starts at 0 with lines every 20 x-scale units
  // Y grid starts at -512 with lines every 64 y-scale units
  // blue grid
  gr.setGraphGrid(gxLow, 20.0, gyLow, 64.0, TFT_BLUE);
  // Draw empty graph, top left corner at pixel coordinate 40,10 on TFT
  gr.drawGraph(10, 20);
  // Start a trace with using red, trace points are in x and y scale units
  // In this example a horizontal line is drawn
  tr.startTrace(TFT_RED);
  // Add a trace point at 0.0,0.0 on graph
  // Create Text
  maxAcc.createSprite(200,10);
  HallTemp.createSprite(120,10);
}

//read Temp/Accelerator/InputVoltage
float readMPU(int param) {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  switch(param){
    case 0:
      return g.gyro.x - 0.8;
    case 1:
      return g.gyro.y;
    case 2:
      return g.gyro.z;
    case 3:
      return a.acceleration.x;
    case 4:
      return a.acceleration.y;
    case 5:
      return a.acceleration.z;
    case 6:
      return temp.temperature;
    case 7:
      sensors.requestTemperatures();
      return   sensors.getTempC(Thermometer);
    case 8:  
      return (analogReadMilliVolts(36)*5.8/1000);
    default:
      return 0;    
  }
}

// Create graph from mpu Moves 

void graphMPU(byte graphType) {
  
  static uint32_t plotTime = millis();
  static float gx = 0.0;
  byte gType = 0;
  byte gScale = 1;
  (graphType)?gScale = 100 : gScale = 50;
  (graphType)?gType = 0 : gType = 3;
  if (abs(readMPU(gType+0)*gScale) >= abs(maxgx)) maxgx = readMPU(gType+0)*gScale;
  if (abs(readMPU(gType+1)*gScale) >= abs(maxgy)) maxgy = readMPU(gType+1)*gScale;
  if (abs(readMPU(gType+2)*gScale) >= abs(maxgz)) maxgz = readMPU(gType+2)*gScale;

  // Create a new plot point every 100ms
  if (millis() - plotTime >= 50) {
    plotTime = millis();
    maxAcc.setTextColor(TFT_WHITE);
    maxAcc.fillScreen(TFT_BLACK);
    maxAcc.setTextSize(1);
    maxAcc.drawString("Max X:"+String(maxgx)+ " Y:" + String(maxgy) + " Z:" + String(maxgz),0,0);
    maxAcc.pushSprite(30,190);
    HallTemp.setTextColor(TFT_WHITE);
    HallTemp.fillScreen(TFT_BLACK);
    HallTemp.setTextSize(1);
    HallTemp.drawString("Temp:"+String(readMPU(8)),0,0);
    HallTemp.pushSprite(75,10);
    // Add a plot, first point in a trace will be a single pixel (if within graph area)
    tr.addPoint(gx, readMPU(gType+0)*gScale);
    tr1.addPoint(gx, readMPU(gType+1)*gScale);
    tr2.addPoint(gx, readMPU(gType+2)*gScale);
    gx += 1.0;
    // If the end of the graph x ais is reached start a new trace at 0.0,0.0
    if (gx > gxHigh) {
      gx = 0.0;
      // gy = 0.0;
      maxgx=0.0;
      maxgy=0.0;
      maxgz=0.0;      
      // Draw empty graph at 40,10 on display to clear old one
      gr.drawGraph(10, 20);
      // Start new trace
      tr.startTrace(TFT_WHITE);
      tr.addPoint(0.0, 0.0);
      tr.addPoint(100.0, 0.0);
      tr.addPoint(0.0, 5.0);
      tr.addPoint(100.0, 5.0);
      tr.startTrace(TFT_RED);
      tr1.startTrace(TFT_GREEN);
      tr2.startTrace(TFT_GOLD);
    }
  }
}

// MAIN LOOP -- runs continuously after setup() ----------------------------
void loop() {
  // graphMPU(0);
  updateEye();
}
