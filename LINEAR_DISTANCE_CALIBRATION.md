# Linjär Distans-kalibrering (För Växellådade Hjul)

Eftersom hjulen inte kan roteras för hand pga växellådan, använder vi ett linjärt distanstest istället.

## Aktuella Värden
- Hjuldiameter: 82.5mm (uppmätt)
- Wheelbase: 244mm (uppmätt)
- TICKS_PER_REV: **310** (BEHÖVER KALIBRERAS!)
- Interrupt mode: **CHANGE** (2X quadrature decoding - ny kod)

## Teoretisk Beräkning

Med aktuella värden:
```
METERS_PER_TICK = (PI * 0.0825) / 310 = 0.000836 m/tick

För 1 meter körsträcka:
Expected ticks per hjul = 1000mm / 0.836mm = 1196 ticks
```

## Kalibrerings-procedur

### Förberedelser:
1. Hitta en rak körbana (minst 1.5 meter lång)
2. Ha ett måttband eller linjal redo
3. Ha maskeringstejp för att märka start/slut-positioner

### Steg 1: Ladda Upp Ny Firmware (VIKTIG!)
Den nya firmware har 2X quadrature decoding som fångar DUBBELT så många encoder-counts:

```bash
# Koppla ur ESP32 från USB
# Koppla in Arduino Uno via USB
cd "/Users/magnus/Documents/PlatformIO/Projects/Round_robot/Arduino Motor control"
pio run --target upload

# Vänta tills uppladdningen är klar
# Koppla ur Arduino, koppla tillbaka ESP32
```

### Steg 2: Kör Testet

1. **Öppna UI**: http://localhost:5001

2. **Nollställ encoders**:
   - Klicka "Zero Encoders & Reset" knappen
   - Bekräfta

3. **Märk startposition**:
   - Sätt en tejp-bit framför roboten (vid framkant)

4. **Kör roboten framåt EXAKT 1 meter**:
   - Använd joysticken i UI
   - Kör långsamt och rakt framåt
   - Stoppa när roboten har kört EXAKT 1000mm (använd måttband)
   - Tips: Lägg ut måttband på golvet först, märk 1-meter punkt med tejp

5. **Läs av Encoder Counts**:
   - Titta på "Odometry" panelen i UI
   - Anteckna:
     - **Encoder Left**: __________ ticks
     - **Encoder Right**: __________ ticks
     - **Medelvärde**: (Left + Right) / 2 = __________ ticks

### Steg 3: Beräkna Korrekt TICKS_PER_REV

Använd medelvärdet från steget ovan:

```
TICKS_PER_REV = Measured_ticks * (PI * 82.5) / 1000

Exempel:
Om du mätte 1300 ticks medelvärde:
TICKS_PER_REV = 1300 * 259.18 / 1000 = 336.9 ≈ 337
```

**Formel (enklare):**
```
TICKS_PER_REV = Measured_ticks * 0.2592
```

### Steg 4: Uppdatera Firmware

1. Öppna filen:
   ```
   Arduino Motor control/src/main.cpp
   ```

2. Hitta rad 37:
   ```cpp
   const float TICKS_PER_REV = 310.0f;  // GAMMAL VALUE
   ```

3. Uppdatera med ditt beräknade värde:
   ```cpp
   const float TICKS_PER_REV = 337.0f;  // DIT NYA VÄRDE (exempel)
   ```

4. Kompilera och ladda upp igen:
   ```bash
   cd "/Users/magnus/Documents/PlatformIO/Projects/Round_robot/Arduino Motor control"
   pio run --target upload
   ```

5. Uppdatera också server config:
   ```
   robot-ui/server/robot_config.json
   ```
   Ändra:
   ```json
   "ticks_per_revolution": 337
   ```

6. Starta om servern

### Steg 5: Verifiera Kalibrering

1. Nollställ encoders igen
2. Kör roboten 1 meter framåt igen
3. Kontrollera "Host Odometry" i UI:
   - X: Ska visa ~1.000 m (±0.01m är okej)
   - Heading: Ska vara ~0° (robot körde rakt)

4. Om X visar:
   - **< 1.0m**: TICKS_PER_REV är för HÖGT, minska värdet
   - **> 1.0m**: TICKS_PER_REV är för LÅGT, öka värdet
   - Justera med 2-3% i taget

## Förväntat Resultat efter 2X Decoding

Med den nya quadrature koden (CHANGE interrupt):
- **Gamla RISING only**: ~155 counts per hjulvarv
- **Nya 2X CHANGE**: ~310 counts per hjulvarv (DUBBELT!)

Det betyder odometrin kan visa DUBBELT så många counts som innan!

## Felsökning

**Problem**: "Odometry visar 2 meter fast roboten körde 1 meter"
- **Orsak**: TICKS_PER_REV är för lågt
- **Lösning**: Dubbla värdet (t.ex. 310 → 620)

**Problem**: "Odometry visar 0.5 meter fast roboten körde 1 meter"
- **Orsak**: TICKS_PER_REV är för högt
- **Lösning**: Halvera värdet (t.ex. 310 → 155)

**Problem**: "Robot kör inte rakt, svänger lite"
- **Orsak**: Hjulen har olika friktion ELLER olika diameter
- **Lösning**: Detta fixas EFTER encoder-kalibrering med per-wheel diameter tuning

## Nästa Steg Efter Encoder-kalibrering

När TICKS_PER_REV är korrekt kan vi:
1. Fine-tuning av wheelbase (om rotation inte stämmer)
2. Fine-tuning av per-wheel diameter (om robot inte kör rakt)
3. Testa closed-loop körning och verifiera att robot kommer tillbaka till origin
