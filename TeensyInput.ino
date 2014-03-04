/*-----------------------------------------------------------------------------

Teensy Input
by Allen C. Huffman (alsplace@pobox.com)

Monitor digital inputs, then emit a USB keyboard character.

Pin 11 is reserved for blinking the onboard LED as a heartbeat "we are alive"
indicator.

This software was written to allow a Teensy 2.0 to interface between switches
and a USB keyboard input on a computer.

2013-06-26 0.0 allenh - Initial version, based on my icade_teensy code.

-----------------------------------------------------------------------------*/
#define VERSION "0.0"
//#define LED_OFF

/*
  The following I/O pins will be used as digital inputs
  for each specific switch.
*/
#define SWITCH1_PIN 0
#define SWITCH2_PIN 1
#define SWITCH3_PIN 2
#define SWITCH4_PIN 3
#define BTN1_PIN    4
#define BTN2_PIN    5
#define BTN3_PIN    6
#define BTN4_PIN    7
#define BTN5_PIN    8
#define BTN6_PIN    9
#define BTN7_PIN    10
#define BTN8_PIN    12

/*
  The following keys are the keyboard character sent when each
  input pin is switched.
*/
#define SWITCH1_KEY "1"
#define SWITCH2_KEY "2"
#define SWITCH3_KEY "3"
#define SWITCH4_KEY "4"
#define BTN1_KEY    "A"
#define BTN2_KEY    "B"
#define BTN3_KEY    "C"
#define BTN4_KEY    "D"
#define BTN5_KEY    "E"
#define BTN6_KEY    "F"
#define BTN7_KEY    "G"
#define BTN8_KEY    "H"

#define DI_PIN_COUNT  12   // 12 pins used.
#define DI_PIN_START  1    // First I/O pin.
#define DI_PIN_END    20   // Last I/O pin.

byte myPins[DI_PIN_COUNT] =
  {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN, SWITCH4_PIN,
  BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN,
  BTN5_PIN, BTN6_PIN, BTN7_PIN, BTN8_PIN};

char keymap[][DI_PIN_COUNT] =
  {SWITCH1_KEY, SWITCH2_KEY, SWITCH3_KEY, SWITCH4_KEY,
  BTN1_KEY, BTN2_KEY, BTN3_KEY, BTN4_KEY,
  BTN5_KEY, BTN6_KEY, BTN7_KEY, BTN8_KEY};

char keyDesc[][DI_PIN_COUNT] =
  {"Sw1", "Sw2", "Sw3", "Sw4",
  "Btn1", "Btn2", "Btn3", "Btn4",
  "Btn5", "Btn6", "Btn7", "Btn8"};

/* We want a very short debounce delay for an arcade controller. */
#define DI_DEBOUNCE_MS 10 // 100ms (1/10th second)

#define LED_PIN 11
#define LEDBLINK_MS 1000

/*---------------------------------------------------------------------------*/

/* For I/O pin status and debounce. */
unsigned int  digitalStatus[DI_PIN_COUNT];          // Last set PIN mode.
unsigned long digitalDebounceTime[DI_PIN_COUNT];    // Debounce time.
//unsigned long digitalCounter[DI_PIN_COUNT];         // Times button pressed.
unsigned int  digitalDebounceRate = DI_DEBOUNCE_MS; // Debounce rate.

/* For the blinking LED (heartbeat). */
unsigned int  ledStatus = LOW;             // Last set LED mode.
unsigned long ledBlinkTime = 0;            // LED blink time.
unsigned int  ledBlinkRate = LEDBLINK_MS;  // LED blink rate.

unsigned int pinsOn = 0;

/*---------------------------------------------------------------------------*/

void setup()
{
  // Just in case it was left on...
  //wdt_disable();

  // Initialize the serial port.
  Serial.begin(9600);

  // Docs say this isn't necessary for Uno.
  //while(!Serial) { }

  showHeader();
  
  // Initialize watchdog timer for 2 seconds.
  //wdt_enable(WDTO_4S);

  // LOW POWER MODE!
  // Pins default to INPUT mode. To save power, turn them all to OUTPUT
  // initially, so only those being used will be turn on. See:
  // http://www.pjrc.com/teensy/low_power.html
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    pinMode(thisPin, OUTPUT);
  }

  // Disable Unused Peripherals
  ADCSRA = 0;

  // Initialize the pins and digitalPin array.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Set pin to be digital input using pullup resistor.
    pinMode(myPins[thisPin], INPUT_PULLUP);
    // Set the current initial pin status.
    digitalStatus[thisPin] = HIGH; //digitalRead(thisPin+DI_PIN_START);
    // Clear debounce time.
    digitalDebounceTime[thisPin] = 0;
    //digitalCounter[thisPin] = 0;
  }

  // Set LED pin to output, since it has an LED we can use.
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Ready.");
}

/*---------------------------------------------------------------------------*/

void loop()
{
  // Tell the watchdog timer we are still alive.
  //wdt_reset();

#ifndef LED_OFF
  // LED blinking heartbeat. Yes, we are alive.
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    if (ledStatus==LOW)  // If LED is LOW...
    {
      ledStatus = HIGH;  // ...make it HIGH.
    } else {
      ledStatus = LOW;   // ...else, make it LOW.
    }
    // Set LED pin status.
    if (pinsOn==0) digitalWrite(LED_PIN, ledStatus);
    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+ledBlinkRate;
  }
#endif

  // Check for serial data.
  if (Serial.available() > 0) {
    // If data ready, read a byte.
    int incomingByte = Serial.read();
    // Parse the byte we read.
    switch(incomingByte)
    {
      case '?':
        showStatus();
        break;
      default:
        break;
    }
  }
  
  /*-------------------------------------------------------------------------*/

  // Loop through each Digital Input pin.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    unsigned int status = digitalRead(myPins[thisPin]);

    // In pin status has changed from our last toggle...
    if (status != digitalStatus[thisPin])
    {
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (digitalDebounceTime[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        digitalDebounceTime[thisPin] = millis()+digitalDebounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (digitalDebounceTime[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status==LOW)
            {
              // Emit BUTTON PRESSED string.
              Serial.print(keyDesc[thisPin]);
              Serial.print(" pressed  (sending ");
              Serial.print(keymap[thisPin][0]);
              Serial.println(" to computer).");
              Keyboard.println(keymap[thisPin][0]);
              //digitalCounter[thisPin]++;
              pinsOn++;
#ifndef LED_OFF
              digitalWrite(LED_PIN, HIGH);
#endif
            } else {
              // Emit BUTTON RELEASED string.
              Serial.print(keyDesc[thisPin]);
              Serial.println(" released.");
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            digitalStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            digitalDebounceTime[thisPin] = 0;
        } // End of if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        
      } // End of if (digitalDebounceTime[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      digitalDebounceTime[thisPin] = 0;
    }
  } // End of (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
}

/*---------------------------------------------------------------------------*/

void showHeader()
{
  int i;
  // Emit some startup stuff to the serial port.
  Serial.println();
  Serial.print("teensy_input ");
  Serial.print(VERSION);
  Serial.println(" by Allen C. Huffman (alsplace@pobox.com)");
  Serial.print(DI_PIN_COUNT);
  Serial.print(" DI Pins (");
  for (i=0; i<DI_PIN_COUNT; i++)
  {
    Serial.print(myPins[i]);
    Serial.print("=");
    Serial.print(keyDesc[i]);
    Serial.print(" ");
  }
  Serial.print("), ");
  Serial.print(digitalDebounceRate);
  Serial.println("ms Debounce.");
}

/*---------------------------------------------------------------------------*/

void showStatus()
{
  showDigitalInputStatus();
}

/*---------------------------------------------------------------------------*/

void showDigitalInputStatus()
{
  Serial.print("DI: ");

  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    Serial.print(keyDesc[thisPin]);
    Serial.print("=");
    Serial.print(digitalRead(myPins[thisPin]));
    Serial.print(" ");
    //Serial.print(" (");
    //Serial.print(digitalCounter[thisPin]);
    //Serial.print(") ");
  }
  Serial.println("");
}

/*---------------------------------------------------------------------------*/

// End of file.

