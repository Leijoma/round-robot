# Loopback Testing Guide

## What is Loopback Mode?

Med A0 och A1 ihopkopplade (loopback) kommer allt som Arduino skickar på TX (A1) att tas emot på RX (A0).

Detta är användbart för att:
- ✓ Verifiera att SoftwareSerial fungerar
- ✓ Testa att Arduino kan ta emot sina egna meddelanden
- ✓ Debugga protokoll-implementation

## Current Status

**✅ Debug Output Working**
```
=== Arduino Motor Control Firmware ===
Version: 1.0
Debug enabled on USB Serial (115200)
RobotLink on SoftwareSerial A0/A1 (9600)
...
=== Setup Complete ===
Ready for commands on A0/A1 (9600 baud)
```

**⏳ Waiting for Commands**

Arduino väntar på kommandon men skickar inget eftersom:
- Odometry streaming är disabled som standard
- Inga kommandon har skickats än

## Problem med Full Loopback

Med A0 direkt kopplad till A1:
- ✓ Arduino kan ta emot vad den skickar
- ✗ MEN vi kan inte enkelt skicka kommandon utifrån
- Allt vi skickar kommer också att loopas tillbaka

## Lösningar

### Option 1: Halvduplex med brytare
```
         Switch
A0 (RX) ---o o--- A1 (TX)
             |
          Externa
         kommandon
```

1. Koppla loss loopback
2. Skicka kommando utifrån (ENABLE_STREAM)
3. Koppla tillbaka loopback
4. Arduino skickar ODOM som loopas tillbaka

### Option 2: Extern USB-Serial adapter
```
USB-Serial Adapter (FTDI/CP2102)
  TX --> A0 (RX)
  RX <-- A1 (TX)
  GND -- GND
```

Detta är bästa lösningen för verklig användning.

### Option 3: Testa utan loopback (Rekommenderat)
```
USB-Serial Adapter
  TX --> A0 (RX)    <-- Skicka kommandon
  RX <-- A1 (TX)    <-- Ta emot odometry
  GND -- GND

USB (Arduino)       <-- Se debug output
```

## Snabb Test: Aktivera Streaming från Kod

Istället för loopback, ändra firmware tillfälligt för att auto-aktivera streaming:

```cpp
// I setup(), efter "Initialize timing":
streamEnabled = true;
streamInterval = 100;  // 100ms = 10 Hz
Serial.println(F("Auto-enabled streaming for test"));
```

Då kommer Arduino att skicka ODOM kontinuerligt, och med loopback kommer den se sina egna meddelanden.

## Köra Test Med Nuvarande Setup

### 1. Visa Debug Output
```bash
python3 debug_monitor.py
```

Du bör se:
```
=== Setup Complete ===
Ready for commands on A0/A1 (9600 baud)
```

### 2. Vad Händer Nu?
- Arduino lyssnar på A0 (RX)
- Men skickar inget på A1 (TX) än
- Därför ser vi inga "RX:" meddelanden

### 3. För att Testa Loopback
Du behöver antingen:

**A) Modifiera firmware** (enklast för test):
```cpp
// main.cpp, rad ~586 i setup():
streamEnabled = true;
streamInterval = 100;
```
Rebuild och upload. Arduino kommer då skicka ODOM var 100ms.

**B) Använd extern USB-serial** på A0/A1 för att skicka ENABLE_STREAM kommando.

## Verifiering

Om loopback fungerar kommer du se:

```
[DEBUG] RX: Type=0x01 Len=18
[DEBUG] RX: Type=0x01 Len=18
[DEBUG] RX: Type=0x01 Len=18
...
```

Typ 0x01 = ODOM message. Detta visar att:
1. Arduino skickar ODOM på A1 (TX)
2. Det loopas till A0 (RX)
3. Arduino tar emot sitt eget meddelande
4. Debug visar att det kom in

## Rekommendation

För verklig användning:
1. **Ta bort loopback** (koppla loss A0 från A1)
2. **Anslut ESP32 eller FTDI** till A0/A1
3. **Använd test_robot.py** med rätt port @ 9600 baud

Då kommer du ha:
- USB = Debug output (se vad som händer)
- A0/A1 = RobotLink kommandon och odometry

## Nuvarande Status

✅ Firmware fungerar
✅ Debug output fungerar
✅ SoftwareSerial initierad
⏳ Väntar på kommandon ELLER auto-enable streaming

Vill du att jag ändrar firmware för att auto-aktivera streaming så vi kan testa loopback?
