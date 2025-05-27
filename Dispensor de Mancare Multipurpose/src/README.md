# Descrierea Aplicației (Firmware) - Dispenser Automat de Hrană

## Mediu de dezvoltare
- **AVR-GCC** cu toolchain standard pentru microcontrolere AVR
- **PlatformIO** și **Arduino IDE** compatible
- **Target**: ATmega328P (Arduino Uno/Nano)
- **Frecvență**: 16MHz crystal oscillator

## Biblioteci și surse 3rd-party folosite
- **avr/io.h** - registre și definții hardware AVR
- **avr/interrupt.h** - gestionare întreruperi
- **util/delay.h** - funcții de întârziere precisă
- **stdint.h** - tipuri de date standard

**Implementare proprie** (fără dependințe externe):
- Protocol I2C low-level pentru comunicația cu LCD-ul
- Control PWM manual pentru servomotor SG90
- Interfață UART pentru debugging și monitorizare

## Algoritmi și structuri implementate

### 1. **Structuri de date**
```c
typedef enum {
    MODE_HUMAN = 0,    // Mod pentru hrănirea umană (cu opțiune fără cooldown)
    MODE_ANIMAL        // Mod pentru hrănirea animale (cooldown obligatoriu)
} DispenseMode;

typedef enum {
    FOOD_LOW = 0,      // 5 cicluri de dispensare
    FOOD_MEDIUM,       // 10 cicluri de dispensare  
    FOOD_HIGH          // 15 cicluri de dispensare
} FoodAmount;

// Setări cooldown pentru umani (în minute) - cu opțiunea NONE
const uint16_t humanCooldowns[] = {0, 5, 10, 15, 30, 45, 60};
#define HUMAN_COOLDOWN_COUNT 7

// Setări cooldown pentru animale (în minute) - fără opțiunea NONE
const uint16_t animalCooldowns[] = {30, 60, 120, 240, 480, 720};
#define ANIMAL_COOLDOWN_COUNT 6

// Configurare cicluri de dispensare
const uint8_t foodCycles[] = {5, 10, 15};    // Numărul de cicluri pentru LOW, MEDIUM, HIGH
const uint16_t dispenseTimes[] = {1000, 1000, 1000}; // Timp per ciclu în ms (1 secundă)
#define CYCLE_TIME 1000      // Timp per ciclu în ms

// Variabile globale de sistem
DispenseMode currentMode = MODE_HUMAN;
FoodAmount currentAmount = FOOD_LOW;
uint8_t cooldownIndex = 0;
uint8_t servoActive = 0;
uint8_t cooldownActive = 0;
uint8_t currentCycle = 0;
uint8_t totalCycles = 1;
uint8_t cyclePhase = 0; // 0 = dispensing (open), 1 = closing
```

### 2. **Algoritmi de control**

**Sistem de dispensare multi-ciclu:**
- **LOW**: 5 cicluri × 1 secundă = 5 secunde totale
- **MEDIUM**: 10 cicluri × 1 secundă = 10 secunde totale
- **HIGH**: 15 cicluri × 1 secundă = 15 secunde totale
- **Pattern**: open(1s) → close(instant) → open(1s) → close(instant)...

**Control servo pentru SG90:**
- **Unghi maxim**: 90° (limita servo-ului SG90)
- **Alimentare**: 5V pentru performanță optimă
- **PWM precis**: 1ms-2ms cu 10 repetiții pentru stabilitate
- **Formula**: `pulse_width = 1000 + (angle * 1000) / 90 μs`

**Sistem de cooldown dual:**
- **Umani**: 0 (NONE), 5, 10, 15, 30, 45, 60 minute
- **Animale**: 30, 60, 120, 240, 480, 720 minute (fără opțiunea NONE)
- **Timer software**: bazat pe bucla principală (increment la 10ms)

### 3. **Arhitectura sistemului**

**Mașină de stări:**
1. **IDLE** - sistem pregătit, afișează setări curente
2. **MULTI-CYCLE DISPENSING** - alternează între deschis/închis cu progres vizual
3. **COOLDOWN** - sistem blocat cu countdown afișat pe LCD

**Interfață utilizator:**
- **Button 1**: Mode switching (HUMAN ↔ ANIMAL)
- **Button 2**: Food amount (LOW → MED → HIGH → LOW...)
- **Button 3**: Cooldown time (specific fiecărui mod)
- **LCD 16x2**: Status real-time și progres operațiuni
- **Proximity sensor**: Activare automată prin detectare obiect

## Funcții și module implementate

### **Comunicație I2C**
- `i2c_init()` - configurare TWI la 100kHz
- `i2c_start()`, `i2c_stop()`, `i2c_write()` - protocul I2C complet
- `lcd_send_i2c()` - transmisie către LCD

### **Driver LCD cu I2C expander (HD44780)**
- `lcd_init()` - secvență de inițializare 4-bit
- `lcd_command()`, `lcd_write()` - comenzi și date
- `lcd_print()`, `lcd_print_number()` - funcții utilitare
- `lcd_set_cursor()`, `lcd_clear()` - control cursor și display

### **Control servomotor SG90**
- `servo_set_angle()` - poziționare precisă 0-90°
- **Calibrare pentru SG90**: optimizat pentru 5V
- **PWM manual**: 20ms perioada, 1-2ms lățime puls
- **Stabilitate**: 10 repetiții per comandă

### **Sistem multi-ciclu de dispensare**
- `open_trap()` - inițializează secvența de cicluri
- **Logică în bucla principală** pentru gestionarea fazelor:
  - **cyclePhase 0**: dispensing (servo deschis 1 secundă)
  - **cyclePhase 1**: closing (servo închis instantaneu)
- **Progres LCD**: actualizare în timp real "Cycle: X/Y"

### **Gestionare cooldown**
- **Pentru MODE_HUMAN**: opțiunea "0" = fără cooldown
- **Pentru MODE_ANIMAL**: cooldown obligatoriu (minimum 30 min)
- `get_current_cooldown()` - returnează timpul în funcție de mod
- **Afișare LCD**: "No cooldown" sau "Cooldown: Xm"

### **Interfață utilizator**
- `handle_buttons()` - debouncing și edge detection
- **Reset automat**: cooldown-ul se oprește la schimbarea modului
- `update_display()` - afișare condiționată pe starea sistemului
- **Feedback UART**: logging detaliat pentru debug

### **Detectare proximitate**
- `check_proximity()` - citire senzor cu pull-up intern
- **Edge detection**: activare pe front descrescător (1→0)
- **Filtrare zgomot**: prin variabila `lastProximityState`

## Algoritm principal

```c
// Bucla principală - gestionare completă sistem
while (1) {
    handle_buttons();                    // Procesează interfața utilizator
    
    if (!cooldownActive && !servoActive) {
        // Detectare proximitate doar când sistemul este gata
        if (edge_detected) open_trap();
    }
    
    if (servoActive) {
        // Cicluri de dispensare cu timing precis
        if (cyclePhase == 0) {          // Dispensing phase
            if (time > 1_second) → close_servo, switch_to_closing
        } else {                        // Closing phase  
            increment_cycle;
            if (cycle <= total) → open_servo, restart_dispensing
            else → finish_all_cycles
        }
    }
    
    if (cooldownActive) {
        // Countdown cu afișare progres
        if (cooldown_expired) → system_ready
    }
    
    delay_ms(10);  // Control rata execuție: 100Hz
}
```

## Caracteristici ale sistemului

### **Flexibilitate operațională**
- **Modul HUMAN**: acces rapid fără cooldown pentru testare/demonstrații
- **Modul ANIMAL**: control strict cu cooldown-uri lungi pentru programare
- **Comutare instantanee** între moduri cu reset automat al stării

### **Performanță**
- **Închidere instantanee** între cicluri (fără întârzieri inutile)
- **Timer software eficient** pentru operațiuni lungi
- **Gestionare precisă** a resurselor fără întreruperi
- **Rata execuție**: 100Hz pentru răspuns rapid

### **Siguranță și robustețe**
- **Reset automat** la schimbarea modurilor
- **Debouncing hardware și software** pentru toate butoanele
- **Sistem fail-safe** cu monitorizare UART continuă
- **Validare intrări** și limitări hardware

### **Interfață completă**
- **Feedback vizual**: LCD cu progres în timp real
- **Monitoring remot**: UART logging pentru toate operațiunile
- **Control intuitiv**: 3 butoane pentru acces la toate funcțiile
- **Diagnosticare**: mesaje detaliate pentru troubleshooting

## Specificații tehnice

- **Microcontroller**: ATmega328P @ 16MHz
- **Servomotor**: SG90 90° (0-90° operațional)
- **Display**: LCD 16x2 cu I2C (adresa 0x27)
- **Senzor**: Proximity sensor (active LOW)
- **Interfață**: 3 butoane (active LOW cu pull-up)
- **Comunicații**: UART 9600 baud pentru debugging
- **Alimentare**: 5V pentru componente, servo la performanță optimă
- **Memorie**: ~8KB Flash, ~512B RAM utilizate

Sistemul oferă un dispenser automat de hrană complet funcțional, cu control granular al cantității prin sistemul de cicluri multiple și flexibilitate maximă pentru utilizare atât umană cât și pentru animale, prin sistemul dual de cooldown configurable.
