# Motor Control Diagnostic Report

**Datum:** 2026-01-06
**System:** Arduino Uno + Monster Moto Shield + Encoders

## Sammanfattning

✅ **Funger:** Kommunikation, streaming, kommandon når fram
❌ **PROBLEM:** Motorerna rör sig INTE eller rör sig mycket dåligt

---

## Testresultat

### ODOM Streaming
- ✅ ODOM data tas emot perfekt @ 5 Hz (200ms intervall)
- ✅ 150 meddelanden på 30 sekunder
- ✅ ESP32 ↔ Arduino kommunikation fungerar

### Kommando-flöde
- ✅ STOP kommandon når fram (Type=0x1A)
- ✅ SET_VEL kommandon når fram (Type=0x10)
- ✅ SET_PID kommandon når fram (Type=0x11)
- ✅ SET_DEADBAND kommandon når fram (Type=0x12)

### Motor Performance Test

Testad med olika PID och deadband-inställningar:

| Target Vel | Left Motor | Right Motor | Resultat |
|------------|-----------|-------------|----------|
| 0.10 m/s   | 0.000 m/s  | 0.007 m/s   | ❌ Rör sig inte |
| 0.15 m/s   | 0.000 m/s  | 0.000 m/s   | ❌ Rör sig inte |
| 0.20 m/s   | 0.000 m/s  | 0.048-0.129 m/s | ⚠️  Endast höger motor |
| 0.25 m/s   | 0.000 m/s  | 0.003-0.130 m/s | ⚠️  Endast höger motor |
| 0.30 m/s   | 0.000 m/s  | 0.000 m/s   | ❌ Rör sig inte |
| 0.35 m/s   | 0.000 m/s  | 0.000 m/s   | ❌ Rör sig inte |
| 0.40 m/s   | 0.074 m/s  | 0.125 m/s   | ⚠️  Börjar röra sig |
| 0.45 m/s   | 0.156 m/s  | 0.146 m/s   | ⚠️  Rör sig (66% fel) |
| 0.50 m/s   | 0.155 m/s  | 0.147 m/s   | ⚠️  Rör sig (70% fel) |

**Slutsats:**
- **VÄNSTER MOTOR rör sig INTE alls vid låga hastigheter (< 0.40 m/s)**
- **HÖGER MOTOR rör sig sporadiskt och opålitligt**
- Endast vid ≥ 0.45 m/s börjar båda motorerna röra sig
- Prestanda är extremt dålig (66-100% fel)

---

## Möjliga Orsaker

### 1. Hårdvaruproblem (MEST TROLIGT)

#### Motor/Koppling
- ❓ Vänster motor är lös eller inte ordentligt inkopplad
- ❓ Dåliga elkontakter på Monster Moto Shield
- ❓ Lösa kabelkontakter
- ❓ Motor brush/kommutator problem
- ❓ Motorer har för hög friktion/mekanisk belastning

#### H-Bridge (Monster Moto Shield)
- ❓ VNH2SP30 H-bridge för M1 (vänster) fungerar inte korrekt
- ❓ Dålig strömförsörjning
- ❓ Överhettning/skydd aktiverat
- ❓ PWM-insignal når inte H-bridge

#### Encoders
- ❓ Encoder-signaler läses inte korrekt
- ❓ Encoder-kablar lösa eller felaktigt kopplade
- ❓ Encoders är trasiga eller slitna

### 2. Firmware-problem (MINDRE TROLIGT)

- ❓ PWM-värden räknas fel
- ❓ Deadband-kompensation fungerar inte
- ❓ PID output clipping

### 3. Mekaniska Problem

- ❓ Hjul eller drivaxlar är fastlåsta
- ❓ För hög mekanisk friktion i drivlinan
- ❓ Växellåda har problem

---

## Rekommenderade Åtgärder

### Steg 1: Hårdvarutest (KRITISKT!)

1. **Visuell inspektion:**
   - Kontrollera alla kabelanslutningar till motorerna
   - Kontrollera Monster Moto Shield sitter ordentligt på Arduino
   - Leta efter lösa kontakter, kalla lödningar, skadade kablar

2. **Manuell test:**
   - Försök vrida hjulen för hand - är det mycket motstånd?
   - Lyssna efter ovanliga ljud från motorerna
   - Känner Monster Moto Shield varm? (kan indikera överhettning)

3. **Strömförsörjning:**
   - Kolla att batterierna/strömkällan ger tillräckligt med ström
   - Mät spänning på motorutgångarna med multimeter

4. **Direkt PWM-test:**
   Ladda upp `test_motor_hardware.cpp` som testar motorerna direkt med PWM utan PID:
   ```
   - PWM 50: Motor bör snurra långsamt
   - PWM 100: Motor bör snurra snabbare
   - PWM 150: Motor bör snurra rejält
   - PWM 255: Motor bör snurra max hastighet
   ```

### Steg 2: Encoder-test

Skapa ett test som skriver ut encoder-värden i realtid medan du manuellt vrider hjulen.

### Steg 3: Om hårdvaran fungerar

Justera PID och deadband baserat på testresultat:
- Nuvarande deadband (35-40) kan vara för låg
- Testa deadband 60-100
- Testa högre PID-gains (Kp=40-50, Ki=20-25)

---

##  Optimerade Inställningar (Från Test)

**Bästa hittade värden** (men motorerna fungerar fortfarande dåligt):

```cpp
Kp = 30.0
Ki = 15.0
Kd = 0.5
Deadband Forward = 40.0
Deadband Reverse = 40.0
```

**MEN:** Dessa inställningar sparades till EEPROM men ger fortfarande dålig prestanda eftersom problemet är hårdvara, INTE mjukvara.

---

## Fil-översikt

Skapade filer under denna session:

1. **optimize_motor_control.py** - Första optimeringsförsök (misslyckades pga timing)
2. **debug_communication.py** - Visar kommunikationsflöde mellan host/ESP32/Arduino
3. **simple_odom_test.py** - Verifierar att ODOM streaming fungerar (✅ FUNKAR)
4. **optimize_v2.py** - Fullständig optimering med PID/deadband test (✅ KÖRDES)
5. **test_motor_hardware.cpp** - Direkt PWM-test för hårdvara (KÖR DETTA!)

---

## Nästa Steg

1. **KÖR HÅRDVARUTEST FÖRST!**
   - Ladda upp `test_motor_hardware.cpp`
   - Lyssna/titta på motorerna
   - Notera vilka PWM-nivåer som krävs för att motorerna ska börja snurra

2. **Kontrollera fysiskt:**
   - Alla kablar
   - Monster Moto Shield kontakter
   - Motoranslutningar
   - Encoder-anslutningar

3. **Rapportera tillbaka:**
   - Vilket PWM-värde börjar vänster motor snurra vid?
   - Vilket PWM-värde börjar höger motor snurra vid?
   - Finns det några konstiga ljud eller beteenden?

---

**VIKTIG SLUTSATS:** Problemet är INTE PID eller deadband - det är hårdvara! Vänster motor fungerar inte alls, och höger motor kräver mycket högre PWM än förväntat för att börja röra sig.
