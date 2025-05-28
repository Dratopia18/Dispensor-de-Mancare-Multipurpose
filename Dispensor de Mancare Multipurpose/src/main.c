#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// Adresa I2C a LCD-ului
#define LCD_ADDR 0x27

// Definirea pinilor controllerului LCD
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01
#define LCD_D4 0x10
#define LCD_D5 0x20
#define LCD_D6 0x40
#define LCD_D7 0x80
#define LCD_BACKLIGHT 0x08

// Comenzi LCD
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

// Flaguri pentru comenzi
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_ON 0x02
#define LCD_BLINK_ON 0x01
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INC 0x01
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

// === MODIFICARE PENTRU PWM HARDWARE ===
// Schimbăm pinul servo pentru a folosi PWM hardware
#define SERVO_PIN PB1               // OC1A (Arduino digital pin 9) - PWM hardware
#define PROXIMITY_PIN PD4           // Arduino digital pin 4
#define BTN_MODE_PIN PD5            // Button 1: Mode (Human/Animal)
#define BTN_AMOUNT_PIN PD6          // Button 2: Food amount (LOW/MED/HIGH)
#define BTN_COOLDOWN_PIN PD7        // Button 3: Cooldown time

// Constante pentru servo cu PWM hardware
#define SERVO_OPEN_ANGLE 90         // Unghi pentru deschis (90 grade)
#define SERVO_CLOSE_ANGLE 0         // Unghi pentru închis (0 grade)

// Moduri de funcționare
typedef enum {
    MODE_HUMAN = 0,
    MODE_ANIMAL
} DispenseMode;

// Cantități de hrană
typedef enum {
    FOOD_LOW = 0,    // 5 cicluri de dispensare
    FOOD_MEDIUM,     // 10 cicluri de dispensare
    FOOD_HIGH        // 15 cicluri de dispensare
} FoodAmount;

// Setări cooldown pentru oameni (în minute) - cu opțiunea "no cooldown"
const uint16_t humanCooldowns[] = {0, 5, 10, 15, 30, 45, 60};
#define HUMAN_COOLDOWN_COUNT 7

// Setări cooldown pentru animale (în minute)
const uint16_t animalCooldowns[] = {30, 60, 120, 240, 480, 720};
#define ANIMAL_COOLDOWN_COUNT 6

// Setări pentru cicluri multiple
const uint8_t foodCycles[] = {5, 10, 15};    // Numărul de cicluri pentru LOW, MEDIUM, HIGH
#define CYCLE_TIME 1000      // Timp per ciclu în ms (1 secundă)

// Variabile globale de setări
DispenseMode currentMode = MODE_HUMAN;
FoodAmount currentAmount = FOOD_LOW;
uint8_t cooldownIndex = 0;

// Variabile globale de sistem
uint8_t _backlight;
uint8_t servoActive = 0;
uint8_t cooldownActive = 0;
uint32_t servoStartTime = 0;
uint32_t cooldownStartTime = 0;
uint8_t lastProximityState = 1;

// Variabile pentru cicluri multiple
uint8_t currentCycle = 0;
uint8_t totalCycles = 1;
uint8_t cyclePhase = 0; // 0 = dispensing (open), 1 = closing

// Declarații înainte - TOATE funcțiile
void servo_set_angle_hardware(uint8_t angle);
void servo_init_hardware(void);
void open_trap(void);
void close_trap(void);
void handle_buttons(void);
void update_display(void);
uint16_t get_current_cooldown(void);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
void lcd_send_i2c(uint8_t data);
void lcd_pulse_enable(uint8_t data);
void lcd_write_4bits(uint8_t value);
void lcd_command(uint8_t command);
void lcd_write(uint8_t data);
void lcd_print_number(uint16_t num);
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);
void uart_init(uint32_t baud);
void uart_putchar(char c);
void uart_print(const char *str);
void delay_ms(uint16_t ms);
void buttons_init(void);
void proximity_init(void);
uint8_t check_proximity(void);

// === PWM HARDWARE PENTRU SERVO (CONFORM LAB 3) - CU FIX PENTRU LCD ===

void servo_enable_pwm(void) {
    // Activează ieșirea PWM pe OC1A
    TCCR1A |= (1 << COM1A1);  // Non-inverting mode
}

void servo_disable_pwm(void) {
    // Dezactivează ieșirea PWM pe OC1A
    TCCR1A &= ~(1 << COM1A1);
    // Setează pin-ul manual la LOW pentru a opri servo-ul
    PORTB &= ~(1 << SERVO_PIN);
}

void servo_init_hardware(void) {
    // Configurare pin servo ca output (OC1A = PB1 = Arduino pin 9)
    DDRB |= (1 << SERVO_PIN);
    
    // === CONFIGURARE TIMER1 PENTRU PWM HARDWARE ===
    
    // Configurare Fast PWM cu ICR1 ca TOP - DAR PWM DEZACTIVAT INIȚIAL
    // WGM1[3:0] = 1110 (Fast PWM, TOP = ICR1)
    TCCR1A = (1 << WGM11);            // WGM11 = 1, DAR fără COM1A1 încă
    
    TCCR1B = (1 << WGM13) |           // WGM13 = 1 (parte din Fast PWM mode 14)
             (1 << WGM12) |           // WGM12 = 1 (parte din Fast PWM mode 14)
             (1 << CS11);             // Prescaler = 8 (CS1[2:0] = 010)
    
    // Setează TOP pentru frecvența de 50Hz (perioada 20ms)
    ICR1 = 39999;  // TOP value pentru 50Hz
    
    // Setează pin-ul manual la LOW (servo în repaus)
    PORTB &= ~(1 << SERVO_PIN);
    
    // Inițializare servo la poziția închis cu PWM hardware
    servo_set_angle_hardware(SERVO_CLOSE_ANGLE);
    
    uart_print("Servo initialized - PWM ready\r\n");
}

void servo_set_angle_hardware(uint8_t angle) {
    // Limitare unghi la 0-90 grade pentru servo SG90
    if (angle > 90) angle = 90;
    
    // Calculează valoarea OCR1A pentru unghiul dorit
    // Pentru servo SG90: 1ms = 0°, 1.5ms = 45°, 2ms = 90°
    uint16_t ocr_value = 2000 + ((uint32_t)angle * 2000) / 90;
    OCR1A = ocr_value;
    
    // Activează PWM și trimite 10 pulsuri pentru stabilizare rapidă
    // (similar cu PWM manual care făcea 10 repetări)
    servo_enable_pwm();
    
    // 10 pulsuri la 50Hz = 10 * 20ms = 200ms
    // Aceasta este echivalentul celor 10 repetări din PWM manual
    delay_ms(200);  // Timpul necesar pentru 10 pulsuri PWM
    
    // Dezactivează PWM după stabilizare
    servo_disable_pwm();
}

// === FUNCȚII I2C ===

void i2c_init() {
    // Setează viteza la 100kHz (pentru cristal de 16MHz)
    TWSR = 0;
    TWBR = 72;  // ((16000000L/100000L)-16)/2
    
    // Activează interfața TWI
    TWCR = (1<<TWEN);
}

void i2c_start() {
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop() {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(50);  // Așteaptă să fie procesată
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

// === FUNCȚII LCD ===

void lcd_send_i2c(uint8_t data) {
    i2c_start();
    i2c_write(LCD_ADDR << 1);  // Adresa + Write bit (0)
    i2c_write(data | _backlight);
    i2c_stop();
}

void lcd_pulse_enable(uint8_t data) {
    lcd_send_i2c(data | LCD_EN);  // Enable high
    _delay_us(1);                 // Așteaptă 1us
    lcd_send_i2c(data & ~LCD_EN); // Low
    _delay_us(50);                // Așteaptă 50us
}

void lcd_write_4bits(uint8_t value) {
    lcd_send_i2c(value);
    lcd_pulse_enable(value);
}

void lcd_command(uint8_t command) {
    // Trimite partea superioară
    lcd_write_4bits((command & 0xF0));
    // Trimite partea inferioară
    lcd_write_4bits((command & 0x0F) << 4);
}

void lcd_write(uint8_t data) {
    // Setează RS pentru a indica date
    lcd_write_4bits(((data & 0xF0) | LCD_RS));
    lcd_write_4bits(((data & 0x0F) << 4) | LCD_RS);
}

void lcd_print_number(uint16_t num) {
    char buffer[6];
    uint8_t i = 0;
    
    if (num == 0) {
        lcd_write('0');
        return;
    }
    
    while (num > 0 && i < 5) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }
    
    while (i > 0) {
        lcd_write(buffer[--i]);
    }
}

void lcd_init() {
    _backlight = LCD_BACKLIGHT;  // Pornește backlight-ul
    
    // Așteaptă >40ms după pornire
    _delay_ms(50);
    
    // Inițializare în modul 4-bit
    lcd_write_4bits(0x30);  // Funcție set (8-bit)
    _delay_ms(5);
    lcd_write_4bits(0x30);  // Funcție set (8-bit)
    _delay_us(150);
    lcd_write_4bits(0x30);  // Funcție set (8-bit)
    _delay_us(150);
    lcd_write_4bits(0x20);  // Funcție set (4-bit)
    
    // Modul 4-bit, 2 linii, font 5x8
    lcd_command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
    
    // Display on, cursor off, blink off
    lcd_command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON);
    
    // Curăță display-ul
    lcd_command(LCD_CLEAR_DISPLAY);
    _delay_ms(2);  // Curățarea necesită timp
    
    // Setează modul de intrare: de la stânga la dreapta
    lcd_command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT);
    
    // Deplasează cursorul înapoi la începutul display-ului
    lcd_command(LCD_RETURN_HOME);
    _delay_ms(2);
}

void lcd_clear() {
    lcd_command(LCD_CLEAR_DISPLAY);
    _delay_ms(2);
}

void lcd_home() {
    lcd_command(LCD_RETURN_HOME);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row > 3) {
        row = 0;
    }
    lcd_command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_write(*str++);
    }
}

// === FUNCȚII UART ===

void uart_init(uint32_t baud) {
    uint16_t baud_setting = (F_CPU / 8 / baud - 1) / 2;
    
    // Setează rata baud
    UBRR0H = (uint8_t)(baud_setting >> 8);
    UBRR0L = (uint8_t)baud_setting;
    
    // Activează transmisia și recepția
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    
    // Setează formatul: 8 biți date, 1 bit stop
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

void uart_putchar(char c) {
    // Așteaptă până când buffer-ul de transmisie este gol
    while (!(UCSR0A & (1<<UDRE0)));
    
    // Pune datele în buffer și trimite
    UDR0 = c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

// === FUNCȚII HELPER ===

void delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

// === FUNCȚII LOGICĂ (MODIFICATE PENTRU PWM HARDWARE) ===

void open_trap() {
    servo_set_angle_hardware(SERVO_OPEN_ANGLE);  // ← PWM HARDWARE
    servoActive = 1;
    servoStartTime = 0;  // Resetăm contorul
    
    // Setează numărul de cicluri în funcție de cantitate
    totalCycles = foodCycles[currentAmount];
    currentCycle = 1;
    cyclePhase = 0; // Începe cu dispensing (open)
    
    // Actualizează LCD-ul
    lcd_clear();
    lcd_print("Dispensing...");
    lcd_set_cursor(0, 1);
    lcd_print("Cycle: ");
    lcd_print_number(currentCycle);
    lcd_print("/");
    lcd_print_number(totalCycles);
    
    uart_print("Trap opened - ");
    uart_print(currentAmount == FOOD_LOW ? "LOW (5 cycles)" : 
               currentAmount == FOOD_MEDIUM ? "MEDIUM (10 cycles)" : "HIGH (15 cycles)");
    uart_print("\r\n");
}

void close_trap() {
    servo_set_angle_hardware(SERVO_CLOSE_ANGLE);  // ← PWM HARDWARE
    servoActive = 0;
    
    // Reset variabile pentru cicluri
    currentCycle = 0;
    totalCycles = 1;
    cyclePhase = 0;
    
    // Activează cooldown doar dacă nu este setat pe 0 (no cooldown)
    uint16_t currentCooldownTime = get_current_cooldown();
    if (currentCooldownTime > 0) {
        cooldownActive = 1;
        cooldownStartTime = 0;
        
        // Afișează cooldown pe LCD
        update_display();
        uart_print("Trap closed - Starting cooldown\r\n");
    } else {
        // Pentru cooldown = 0 (no cooldown), nu activăm cooldown-ul
        cooldownActive = 0;
        lastProximityState = 1;  // Reset pentru următoarea detecție
        
        // Afișează setările normale
        update_display();
        uart_print("Trap closed - No cooldown mode\r\n");
    }
}

uint16_t get_current_cooldown() {
    if (currentMode == MODE_HUMAN) {
        return humanCooldowns[cooldownIndex];
    } else {
        return animalCooldowns[cooldownIndex];
    }
}

void buttons_init() {
    // Configurare pini ca intrări cu pull-up intern
    DDRD &= ~((1 << BTN_MODE_PIN) | (1 << BTN_AMOUNT_PIN) | (1 << BTN_COOLDOWN_PIN));
    PORTD |= (1 << BTN_MODE_PIN) | (1 << BTN_AMOUNT_PIN) | (1 << BTN_COOLDOWN_PIN);
}

void handle_buttons() {
    static uint8_t lastBtnState = 0xFF;
    uint8_t btnState = 0;
    
    // Citește starea butoanelor (active LOW)
    if (!(PIND & (1 << BTN_MODE_PIN))) btnState |= 0x01;
    if (!(PIND & (1 << BTN_AMOUNT_PIN))) btnState |= 0x02;
    if (!(PIND & (1 << BTN_COOLDOWN_PIN))) btnState |= 0x04;
    
    // Detectează apăsări noi (tranziții de la HIGH la LOW)
    uint8_t btnPressed = (lastBtnState ^ btnState) & btnState;
    
    // Procesează butoanele apăsate
    if (btnPressed & 0x01) { // Button 1: Mode
        currentMode = (currentMode == MODE_HUMAN) ? MODE_ANIMAL : MODE_HUMAN;
        cooldownIndex = 0; // Reset cooldown index la schimbarea modului
        // Oprește cooldown-ul activ la schimbarea modului
        if (cooldownActive) {
            cooldownActive = 0;
            lastProximityState = 1;
        }
        update_display();
    }
    
    if (btnPressed & 0x02) { // Button 2: Amount
        currentAmount = (currentAmount + 1) % 3;
        update_display();
    }
    
    if (btnPressed & 0x04) { // Button 3: Cooldown
        if (currentMode == MODE_HUMAN) {
            cooldownIndex = (cooldownIndex + 1) % HUMAN_COOLDOWN_COUNT;
        } else {
            cooldownIndex = (cooldownIndex + 1) % ANIMAL_COOLDOWN_COUNT;
        }
        update_display();
    }
    
    lastBtnState = btnState;
}

void update_display() {
    lcd_clear();
    
    if (cooldownActive) {
        lcd_print("Cooldown: ");
        uint16_t remainingMinutes = get_current_cooldown() - (cooldownStartTime / 6000);
        lcd_print_number(remainingMinutes);
        lcd_print("m");
        
        lcd_set_cursor(0, 1);
        if (currentMode == MODE_HUMAN) {
            lcd_print("Mode: HUMAN");
        } else {
            lcd_print("Mode: ANIMAL");
        }
    } else {
        // Prima linie: Mod și Cantitate
        lcd_print(currentMode == MODE_HUMAN ? "HUMAN " : "ANIMAL ");
        lcd_print(currentAmount == FOOD_LOW ? "LOW" : currentAmount == FOOD_MEDIUM ? "MED" : "HIGH");
        
        // A doua linie: Cooldown
        lcd_set_cursor(0, 1);
        uint16_t cooldownTime = get_current_cooldown();
        if (cooldownTime == 0) {
            lcd_print("No cooldown");  // Mesaj special pentru cooldown = 0
        } else {
            lcd_print("Cooldown: ");
            lcd_print_number(cooldownTime);
            lcd_print("m");
        }
    }
}

void proximity_init() {
    // Configurare pin de intrare pentru senzor (doar OUT)
    DDRD &= ~(1 << PROXIMITY_PIN);  // Setează ca intrare
    PORTD |= (1 << PROXIMITY_PIN);  // Activează pull-up intern
    
    uart_print("Proximity sensor initialized on PD4 (Active LOW)\r\n");
}

uint8_t check_proximity() {
    return (PIND & (1 << PROXIMITY_PIN)) ? 1 : 0;
}

// === MAIN (LOGIC IDENTICĂ, DOAR PWM SCHIMBAT) ===

int main(void) {
    // Inițializare componente
    uart_init(9600);
    uart_print("Food Dispenser Starting...\r\n");
    
    i2c_init();
    lcd_init();
    servo_init_hardware();  // ← PWM HARDWARE în loc de manual
    proximity_init();
    buttons_init();
    
    // Afișează setările inițiale
    update_display();
    
    uart_print("System ready\r\n");
    
    // Buclă principală (IDENTICĂ cu originalul)
    while (1) {
        // Procesează butoanele
        handle_buttons();
        
        // Verifică starea senzorului de proximitate doar dacă nu suntem în cooldown
        if (!cooldownActive && !servoActive) {
            uint8_t currentProximityState = check_proximity();
            
            // Detectare front descrescător (tranziție de la 1 la 0) - obiect detectat
            if (!currentProximityState && lastProximityState) {
                uart_print("Object detected!\r\n");
                open_trap();
            }
            
            // Actualizează starea anterioară
            lastProximityState = currentProximityState;
        }
        
        // Verifică ciclurile de dispensare (1s open - instant close pattern)
        if (servoActive) {
            servoStartTime++;
            
            if (cyclePhase == 0) { // Faza de dispensing (servo open pentru 1 secundă)
                if (servoStartTime > (CYCLE_TIME / 10)) {
                    // După 1 secundă, închide servo
                    servo_set_angle_hardware(SERVO_CLOSE_ANGLE);
                    cyclePhase = 1; // Schimbă la closing
                    servoStartTime = 0;
                }
            } else { // Faza de closing (servo closed instantaneu)
                // Închiderea este instantanee, trece IMEDIAT la următorul ciclu
                currentCycle++;
                if (currentCycle <= totalCycles) {
                    // Începe următorul ciclu imediat
                    servo_set_angle_hardware(SERVO_OPEN_ANGLE);
                    cyclePhase = 0; // Înapoi la dispensing (va sta deschis 1 secundă)
                    servoStartTime = 0;
                    
                    // Actualizează LCD cu progresul
                    lcd_clear();
                    lcd_print("Dispensing...");
                    lcd_set_cursor(0, 1);
                    lcd_print("Cycle: ");
                    lcd_print_number(currentCycle);
                    lcd_print("/");
                    lcd_print_number(totalCycles);
                } else {
                    // Toate ciclurile s-au terminat
                    close_trap();
                }
            }
        }
        
        // Gestionare perioadă cooldown (IDENTICĂ)
        if (cooldownActive) {
            cooldownStartTime++;
            
            // Timp cooldown în unități de 10ms
            uint32_t cooldownDuration = (uint32_t)get_current_cooldown() * 60 * 100;
            
            // După ce a trecut perioada de cooldown
            if (cooldownStartTime > cooldownDuration) {
                cooldownActive = 0;
                lastProximityState = 1;
                
                // Afișează setările normale
                update_display();
                
                uart_print("Cooldown complete - System ready\r\n");
            }
        }
        
        // Pauză pentru a controla viteza buclei
        delay_ms(10);
    }
    
    return 0;  // Niciodată executat
}
