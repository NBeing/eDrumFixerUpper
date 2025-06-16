// /////////////////////////////////////////////////////////////////
// // #include "Arduino.h"
// #include "Button2.h"
// #include <Control_Surface.h>
// #include "MINMAX.h"
// #include <WiFi.h> 
// #include <Arduino.h>  // Core Arduino functions (digitalWrite, pinMode, etc.)
// #include <SPI.h>

// #include <U8g2lib.h>

// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif
// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif


// // ============================================
// // NEW: MCP3008 External ADC (8 Channels)
// // ============================================
// // MCP3008 pins
// #define CS_PIN 5
// #define CLOCK_PIN 18
// #define MOSI_PIN 23
// #define MISO_PIN 19
// // #define SDA_PIN 19
// // For 8 drum channels
// #define NUM_CHANNELS 2
// #define THRESHOLD 20      // Lower threshold - cleaner signal
// #define RETRIGGER_MS 30

// // U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ CLOCK_PIN, /* data=*/ MISO_PIN);   // pin remapping with ESP8266 HW I2C
// int lastVal[NUM_CHANNELS] = {0};
// unsigned long lastTrigger[NUM_CHANNELS] = {0};

// MINMAX mm;

// uint32_t start, stop;
// /////////////////////////////////////////////////////////////////
// // USBMIDI_Interface midi; // This is normal midi over usb, but only esp32-s3 has it!
// HairlessMIDI_Interface midi;// This one is for serial midi output
// // HardwareSerialMIDI_Interface midi {Serial2}; // this one is for actual midi jack interface

// bool dbg = false;
// const int numSignals = 5;

// struct DrumPad {
//   Button2 button;
//   byte pin;
//   float signal;
//   float velocity;
//   int id;
//   MINMAX minmax;
//   bool fired;
//   MIDIAddress note;
//   float currentMax;
//   int channel;
//   int baseline;
// };

// struct GlobalState {
//   DrumPad drumPads[numSignals];
//   // byte SignalPins[numSignals] = { 4, 12, 13, 14 };
//   int channel[numSignals] = { 0, 1, 2, 3, 4 };
//   const MIDIAddress notes[numSignals] = {  
//     MIDI_Notes::C[4],
//     MIDI_Notes::D[4],
//     MIDI_Notes::E[4],
//     MIDI_Notes::F[4],
//     MIDI_Notes::G[4]
//   };
// };

// GlobalState globalState;
// // Read single channel from MCP3008
// // This function sends the proper SPI commands to read one ADC channel
// int readMCP3008(int channel) {
//   if (channel < 0 || channel > 7) return 0;
  
//   digitalWrite(CS_PIN, LOW);  // Select the MCP3008
  
//   // Send start bit (1) + single-ended (1) + channel (3 bits)
//   // First byte: 00000001 (start bit at end)
//   // Second byte: 1XXX0000 where XXX is channel number
//   byte commandMSB = 0b00000001;  // Start bit
//   byte commandLSB = 0b10000000 | (channel << 4);  // Single-ended + channel
  
//   // Send commands and read response
//   SPI.transfer(commandMSB);
//   byte msb = SPI.transfer(commandLSB);  // This transfer returns first data bits
//   byte lsb = SPI.transfer(0x00);        // This gets the remaining data bits
  
//   digitalWrite(CS_PIN, HIGH);  // Deselect the MCP3008
  
//   // MCP3008 returns 10 bits: 
//   // msb contains: xxxx xx98 (x = don't care, 9,8 = data bits 9,8)
//   // lsb contains: 7654 3210 (data bits 7-0)
//   return ((msb & 0x03) << 8) | lsb;  // Combine to get 10-bit result (0-1023)
// }

// int readPiezoStable(int ch) {
//   uint32_t sum = 0;
//   int peak = 0;
//   int NUM_SAMPLES = 10;
//   // int PIEZO_PIN = 4;
//   int SAMPLE_DELAY_US = 2;
//   // Take multiple samples
//   for(int i = 0; i < NUM_SAMPLES; i++) {
//     int reading = readMCP3008(ch);
//     sum += reading;
//     if(reading > peak) peak = reading;
//     delayMicroseconds(SAMPLE_DELAY_US);
//   }
  
//   int average = sum / NUM_SAMPLES;
  
//   // For triggers, you might want peak instead of average
//   return peak;
//   // return average;
// }
// void myTapHandler(Button2 &button2Instance) {
//   DrumPad* drumPadPtr = (DrumPad*)button2Instance.getContext();

//   drumPadPtr->fired = true;
//   // Serial.println("Umm i def fired bro " + String(drumPadPtr->fired));
//   // drumPadPtr->velocity = map(drumPadPtr->currentMax , 0, 4095, 0, 127);
//   // drumPadPtr->velocity = 127;
//   if(!dbg){
//     midi.sendNoteOn(drumPadPtr->note, drumPadPtr->velocity);
//   } else {
//     Serial.println("Was tapped");
//   }
// }

// void myReleaseHandler(Button2 &button2Instance) {
//   DrumPad* drumPadPtr = (DrumPad*)button2Instance.getContext();
//   if(!dbg){
//     midi.sendNoteOff(drumPadPtr->note, 0);
//     drumPadPtr->velocity = 0;
//   } else {
//     Serial.println("Release");
//   }
//   drumPadPtr->fired = false;
// }

// /////////////////////////////////////////////////////////////////
// bool drumTriggerReader(void* ctx) {
//   // dereference from void pointer to drumpad
//   // apparently this is dangerous :)
//     DrumPad* drumPadPtr = (DrumPad*)ctx;
//     int reading = readPiezoStable(drumPadPtr->channel);
//     // Serial.print(String(reading) + " ");

//     if (reading > drumPadPtr->baseline + THRESHOLD) { 
//         int velocity = map(reading, drumPadPtr->baseline, 1024, 1, 127);
//         velocity = constrain(velocity, 1, 127);
//         drumPadPtr->velocity = velocity;
//         // Serial.print(String(1) + " ");
//         // Serial.print(String(velocity) + " ");
//         return 1;

//     } else {
//       return 0;
//         // Serial.print(String(0) + " ");
//         // Serial.print(String(0) + " ");
//     }

//   // drumPadPtr->signal = analogRead(drumPadPtr->pin);
//   // drumPadPtr->minmax.add(drumPadPtr->signal);

//   // DrumPad* drumPadPtr = (DrumPad*)ctx;
//   // drumPadPtr->signal = analogRead(drumPadPtr->pin);
//   // drumPadPtr->minmax.add(drumPadPtr->signal);

//   // int _max = drumPadPtr->minmax.maximum();
//   // if(drumPadPtr->signal > 20){
//   //   drumPadPtr->signal = readPiezoStable(drumPadPtr->pin);
//   //   return 1;
//   // } else {
//   //   return 0;
//   // }

//   // if(_max > 20){
//   //   drumPadPtr->minmax.reset();
//   //   for(int i = 0; i < 8; i++ ){
//   //     // delayMicroseconds(10);
//   //     // drumPadPtr->minmax.add(analogRead(drumPadPtr->pin));
//   //   }
//   //   drumPadPtr->currentMax = drumPadPtr->minmax.maximum();
//   //   if(dbg){
//   //     // Serial.println("Returning trigger");
//   //   }
//   //   return 1;
//   // } else {
//   //   if(dbg){
//   //       // Serial.println("Returning off");
//   //   }    
//   //   return 0;
//   // }
// }

// void setup() {
//   analogReadResolution(12);  // 12-bit (0-4095)
//   analogSetAttenuation(ADC_11db);  // 0-3.3V range

//   // Initialize SPI
//   pinMode(CS_PIN, OUTPUT);
//   digitalWrite(CS_PIN, HIGH);  
//   SPI.begin(CLOCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
//   SPI.setFrequency(1350000);  // 1.35MHz for MCP3008

//   // u8g2.begin();
  
//   Serial.begin(115200);
//   delay(1000);
//   Serial2.begin(115200, SERIAL_8N1, 25, 27); 
//   WiFi.mode(WIFI_OFF);
//   WiFi.setSleep(true);
//   // Read baseline for all channels 
//   midi.begin();
  
//   int i = 0;
//   for(DrumPad& drumPad : globalState.drumPads){
//     drumPad.channel = i;
//     drumPad.id = i;
//     // drumPad.pin = globalState.SignalPins[i];
//     drumPad.note = globalState.notes[drumPad.id];
//     drumPad.baseline = readPiezoStable(drumPad.channel);
//     drumPad.minmax.reset();

//     drumPad.button.begin(BTN_VIRTUAL_PIN);
//     drumPad.button.setButtonStateFunction(drumTriggerReader, &drumPad);
//     drumPad.button.setDebounceTime(10);
//     drumPad.button.setTapHandler(myTapHandler);
//     drumPad.button.setReleasedHandler(myReleaseHandler);
//     if(!dbg){
//       midi.sendNoteOff(drumPad.note, 0);
//     } 
//     i++;
//   };
// }

// /////////////////////////////////////////////////////////////////
// void loop() {
//   unsigned long now = millis();  
//   // Scan all 8 channels
//   for(DrumPad& drumPad : globalState.drumPads){
//     // if(drumPad.id > 0 ){
//     //   break;
//     // }
//     drumPad.button.loop();
//     if(dbg){
//       // Serial.print(String(drumPad.signal));
//       // Serial.print(" ");
//       // Serial.print(String(drumPad.fired));
//       // Serial.print(" ");
//       // Serial.print(String(drumPad.velocity));
//       // Serial.print("  ");
//       // Serial.print(String(drumPad.currentMax));
//       // Serial.print("  ");
//     }
//     // int reading = readPiezoStable(drumPad.channel);
//     // Serial.print(String(reading) + " ");

//     // if (reading > drumPad.baseline + THRESHOLD) { 
//     //     int velocity = map(reading, drumPad.baseline, 1024, 1, 127);
//     //     velocity = constrain(velocity, 1, 127);

//     //     Serial.print(String(1) + " ");
//     //     Serial.print(String(velocity) + " ");

//     // } else {
//     //     Serial.print(String(0) + " ");
//     //     Serial.print(String(0) + " ");
//     // }


//     //   // Check retrigger time
//     //   if (now - lastTrigger[ch] < RETRIGGER_MS) continue;
//     //   int reading = readPiezoStable(ch);
//     //   Serial.print(String(reading) + " ");
      
//     //   // Since we're AC coupled, baseline should be ~0
//     //   // Just look for peaks above threshold
//     //   if (reading > THRESHOLD) {
        
//     //     // With 5V reference: 0-2V peaks = 0-400 counts
//     //     // With 3.3V reference: 0-2V peaks = 0-620 counts1
//     //     int velocity = map(reading, THRESHOLD, 300, 1, 127);  // For 5V ref
//     //     velocity = constrain(velocity, 1, 127);
//     //     Serial.print(String(velocity) + " ");
//     //     // Send MIDI note for this drum
//     //     // sendNoteOn(ch, velocity);
//     //     lastTrigger[ch] = now;
        
//     //     // Debug print
//     //     // Serial.print("CH");
//     //     // Serial.print(ch);
//     //     // Serial.print(" Hit! Raw:");
//     //     // Serial.print(String(reading) + " ");
//     //     // Serial.print(" Vel:");
//     //     // Serial.println(velocity);
//     //   } else {
//     //     Serial.print(String(0) + " " );
//     //     Serial.print(String(0) + " " );

//     //   }
//     // } 
//   }
//   // Serial.println();

//   // // Serial.println("Loop!");
//   // // Serial.println(String(millis()));
//   // for(DrumPad& drumPad : globalState.drumPads){
//   //   if(drumPad.signal == 0){
//   //     drumPad.minmax.reset();
//   //   }
//   //   drumPad.button.loop();
//   //   // bool shouldReset = drumPad.signal  < 1;

//   //   if(!dbg){
//   //     // midi.sendNoteOff(notes[0], 0);
//   //   } else {
//   //       // Serial.println(String());
//   //       // float signaltest = analogRead(4);
//   //       // delayMicroseconds(1);
//   //       // signaltest = analogRead(4);

//   //       Serial.println(String(readPiezoStable()));

//   //     // for(int i = 0; i < 1; i++){
//   //       // Serial.println(String(analogRead(4)));

//   //       // Serial.print(String(globalState.drumPads[i].signal));
//   //       // Serial.print(" ");
//   //       // Serial.print(String(globalState.drumPads[i].fired));
//   //       // Serial.print(" ");
//   //       // Serial.print(String(globalState.drumPads[i].velocity));
//   //       // Serial.print("  ");
//   //       // Serial.print(String(globalState.drumPads[i].currentMax));
//   //       // Serial.print("  ");
//   //       // Serial.println();
//   //     // }
//   //   }
//   //   drumPad.fired = false;
//   // };
//     // Serial.println(String(globalState.drumPads[0].signal) + " " + String(globalState.drumPads[0].velocity) + "  " + String(globalState.drumPads[0].fired) +  "  " + String(globalState.drumPads[0].currentMax));
//     // Serial.println("Loop complete");

//     // Serial.println(String(analogRead(4)) + " " + String(globalState.drumPads[0].signal));
//     midi.update();
//     u8g2.clearBuffer();					// clear the internal memory
//     u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
//     u8g2.drawStr(0,10,"hello worllllldddd");	// write something to the internal memory
//     u8g2.sendBuffer();					// transfer internal memory to the display
// }

#include <SPI.h>
#include <Control_Surface.h>  // Your MIDI library
#include <Button2.h>
#include <U8g2lib.h>

// MCP3008 SPI pins
#define CS_PIN 5
#define MCP_CLK_PIN 18
#define MCP_MISO_PIN 19
#define MCP_MOSI_PIN 23

// Timing
#define DEBOUNCE_TIME_MS 50

// Display Setup (example for SSD1306 128x64 I2C)
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ MCP_CLK_PIN, /* data=*/ MCP_MISO_PIN);   // pin remapping with ESP8266 HW I2C

// MIDI Setup (example Serial MIDI)
// USBMIDI_Interface midi; // or your MIDI interface here
HairlessMIDI_Interface midi;// This one is for serial midi output

// Forward declaration
class PiezoPad;

// Pads count
const uint8_t NUM_PADS = 4;

// Global pads array
PiezoPad* pads[NUM_PADS];

// Settings button
Button2 settingsButton(2);  // Example pin 2 for settings button

// State for UI
enum UIState {
  MAIN_PAGE,
  SETTINGS_PAGE
};
UIState currentState = MAIN_PAGE;

// Global variable to track last triggered pad
int lastTriggeredPadIndex = -1;

// =============== MCP3008 read function from you ===============
int readMCP3008(int channel) {
  if (channel < 0 || channel > 7) return 0;
  
  digitalWrite(CS_PIN, LOW);
  
  byte commandMSB = 0b00000001;
  byte commandLSB = 0b10000000 | (channel << 4);
  
  SPI.transfer(commandMSB);
  byte msb = SPI.transfer(commandLSB);
  byte lsb = SPI.transfer(0x00);
  
  digitalWrite(CS_PIN, HIGH);
  
  return ((msb & 0x03) << 8) | lsb;
}

// =============== PiezoPad class with Button2 integration and baseline ============
class PiezoPad {
public:
  uint8_t padId;
  uint8_t midiNote;
  uint8_t midiChannel;
  int adcChannel;

  Button2 button;

  int baseline = 0;
  bool baselineCalibrated = false;

  unsigned long lastTriggerTime = 0;
  bool triggered = false;
  int lastVelocity = 0;
  int lastAdcValue = 0;

  PiezoPad(uint8_t _padId, uint8_t _midiNote, uint8_t _midiChannel, int _adcChannel, uint8_t buttonPin)
    : padId(_padId), midiNote(_midiNote), midiChannel(_midiChannel), adcChannel(_adcChannel), button(buttonPin)
  {
    button.setLongClickDetectedHandler([this](Button2 &b){
      // could do hold action if needed
    });

    button.setPressedHandler([this](Button2 &b){
      // Do nothing on physical button press - actual hit detection via ADC
    });
  }

  void calibrateBaseline() {
    const int NUM_CAL_SAMPLES = 50;
    uint32_t sum = 0;
    for (int i = 0; i < NUM_CAL_SAMPLES; i++) {
      sum += readMCP3008(adcChannel);
      delay(5);
    }
    baseline = sum / NUM_CAL_SAMPLES;
    baselineCalibrated = true;
    Serial.printf("Pad %d baseline calibrated: %d\n", padId, baseline);
  }

  void update() {
    button.loop();

    if (!baselineCalibrated) {
      calibrateBaseline();
      return;
    }

    unsigned long now = millis();
    if (now - lastTriggerTime < DEBOUNCE_TIME_MS) return;

    int adcValue = readPiezoStable(adcChannel, true);
    lastAdcValue = adcValue;

    int hitThreshold = 20; // Tune as needed

    if ((adcValue - baseline) > hitThreshold) {
      int velocity = map(adcValue - baseline, hitThreshold, 1023 - baseline, 1, 127);
      velocity = constrain(velocity, 1, 127);

      midi.sendNoteOn(midiNote, velocity);
      lastVelocity = velocity;
      triggered = true;
      lastTriggerTime = now;
    } else if (triggered) {
      midi.sendNoteOff(midiNote, 0);
      triggered = false;
      lastVelocity = 0;
    }
  }

  void drawInfo(int y) {
    u8g2.setCursor(0, y);
    u8g2.printf("Pad %d: ADC=%d Base=%d Vel=%d", padId, lastAdcValue, baseline, lastVelocity);
  }

private:
  int readPiezoStable(int ch, bool peak_instead_of_average) {
    const int NUM_SAMPLES = 10;
    const int SAMPLE_DELAY_US = 2;
    uint32_t sum = 0;
    int peak = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      int reading = readMCP3008(ch);
      sum += reading;
      if (reading > peak) peak = reading;
      delayMicroseconds(SAMPLE_DELAY_US);
    }
    return peak_instead_of_average ? peak : (sum / NUM_SAMPLES);
  }
};

// =============== Setup and loop ===============

void setup() {
  Serial.begin(115200);
  
  // SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(MCP_CLK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, CS_PIN);

  // Display init
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tr);

  // Initialize pads
  // Example: padId, midiNote, midiChannel, adcChannel, physical button pin
  pads[0] = new PiezoPad(1, 36, 1, 0, 12); // Kick on ADC0, physical button on pin 12
  pads[1] = new PiezoPad(2, 38, 1, 1, 13);
  pads[2] = new PiezoPad(3, 42, 1, 2, 14);
  pads[3] = new PiezoPad(4, 46, 1, 3, 15);

  // Settings button setup
  settingsButton.setPressedHandler([](Button2 &b){
    currentState = (currentState == MAIN_PAGE) ? SETTINGS_PAGE : MAIN_PAGE;
  });

  Serial.println("Setup complete");
}

void loop() {
  settingsButton.loop();

  for (int i = 0; i < NUM_PADS; i++) {
    bool wasTriggeredBefore = pads[i]->triggered;
    pads[i]->update();

    // If pad just triggered, update lastTriggeredPadIndex
    if (!wasTriggeredBefore && pads[i]->triggered) {
      lastTriggeredPadIndex = i;
    }
  }

  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("Last Drum Hit:");

  if (lastTriggeredPadIndex >= 0 && lastTriggeredPadIndex < NUM_PADS) {
    pads[lastTriggeredPadIndex]->drawInfo(25);
  } else {
    u8g2.setCursor(0, 25);
    u8g2.print("No hits yet");
  }

  u8g2.sendBuffer();
}
