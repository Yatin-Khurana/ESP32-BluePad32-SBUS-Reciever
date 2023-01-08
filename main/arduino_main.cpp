/*
    Connect any bluetooth (Bluepad32 Supported) Joypad to an Sbus Input using ESP32 as a Reciver
    
    since the code is built on a weekend only so ofcourse code isn't smart enough
    to invert the SBUS signal, so external inverter is preffered
    or you can just toggle inversion off in ardupilot/px4 code, and use it directly.

    take care of ttl levels maybe.

    Author - Yatin-Khurana
    Released under GNU-GPLV3 Liscence
*/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include <HardwareSerial.h>

#define LED_BUILTIN 2
#define BP32_MAX_GAMEPADS 1

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;
uint8_t playerled = 0;

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

HardwareSerial SerialPort(1);

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;

            digitalWrite(LED_BUILTIN, HIGH);
            delay(300);
            digitalWrite(LED_BUILTIN, LOW);

            // Set LED to White upon connection
            gp->setColorLED(255, 255, 255);

            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            digitalWrite(LED_BUILTIN, HIGH);
            delay(600);
            digitalWrite(LED_BUILTIN, LOW);
            delay(600);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(600);
            digitalWrite(LED_BUILTIN, LOW);
            break;
        }
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        rcChannels[i] = 1500;
    }
    //Serial.begin(100000, SERIAL_8E2);
    SerialPort.begin(100000, SERIAL_8E2, 16, 17);
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];

        if (myGamepad && myGamepad->isConnected()) {
            uint32_t currentMillis = millis();

            rcChannels[0] = map(myGamepad->axisRX(),-511, 512, 1000, 2000);
            rcChannels[1] = map(myGamepad->axisRY(),-511, 512, 1000, 2000);
            rcChannels[2] = map(myGamepad->axisY(),-511, 512, 1000, 2000);
            rcChannels[3] = map(myGamepad->axisX(),-511, 512, 1000, 2000);

            rcChannels[4] = myGamepad->a() ? 2000 : 1000;
            rcChannels[5] = myGamepad->b() ? 2000 : 1000;
            rcChannels[6] = myGamepad->x() ? 2000 : 1000;
            rcChannels[7] = myGamepad->y() ? 2000 : 1000;

            rcChannels[8] = myGamepad->l1() ? 2000 : 1000;
            rcChannels[9] = myGamepad->r1() ? 2000 : 1000;

            rcChannels[10] = map(myGamepad->brake(), 0, 1023, 1000, 2000);
            rcChannels[11] = map(myGamepad->throttle(), 0, 1023, 1000, 2000);

            rcChannels[12] = (myGamepad->dpad() & DPAD_UP)      ? 2000 : 1000;
            rcChannels[13] = (myGamepad->dpad() & DPAD_DOWN)    ? 2000 : 1000;
            rcChannels[14] = (myGamepad->dpad() & DPAD_LEFT)    ? 2000 : 1000;
            rcChannels[15] = (myGamepad->dpad() & DPAD_RIGHT)   ? 2000 : 1000;

            if(myGamepad->a()){
                playerled = 1;
            }
            else if(myGamepad->b()){
                playerled = 2;
            }
            else if(myGamepad->x()){
                playerled = 3;
            }
            else if(myGamepad->y()){
                playerled = 4;
            }

            myGamepad->setPlayerLEDs(playerled & 0x0f);

            if (currentMillis > sbusTime) {
                sbusPreparePacket(sbusPacket, rcChannels, false, false);
                SerialPort.write(sbusPacket, SBUS_PACKET_LENGTH);

                sbusTime = currentMillis + SBUS_UPDATE_RATE;
            }
            delay((int)(SBUS_UPDATE_RATE/3));
        }
    }
}

