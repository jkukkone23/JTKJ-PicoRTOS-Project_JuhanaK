#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tusb.h>
#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
#endif

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1       // toinen CDC-rajapinta (debug/tkjhat)
#define RX_BUFFER_SIZE 128 // vastaanottopuskuri koko
#define MAX_RX_LEN 64      // maksimi pituus vastaanotettavalle viestille
#define BUFFER_SIZE 100    // imu buffer size
#define MORSE_BUF_SIZE 128 // MORSE viestin bufferi

// Tilakoneen esittely ---- lisää puuttuvat tilat tarvittaessa
// TILAT:
// WAITING,         eli odotustila, johon laite menee käynnistyessään
// COLLECTING,      eli viestinkeräystila Pico W:ltä
// MSG_RECEIVED,    eli viesti vastaanotettu tila
// MSG_PRINT,       eli viestin tulostustila (morse-koodina)
// MSG_PRINTED,     eli viestin tulostuksen jälkeinen tila
// MORSE_MSG_SENT,            eli viestinlähetyksen jälkeinen tila, jolloinsoitetaan musiikki
enum state
{
    WAITING = 1,
    COLLECTING,
    MSG_RECEIVED,
    MSG_PRINT,
    MSG_PRINTED,
    MORSE_MSG_SENT
};
enum state programState = WAITING;

volatile char rx_message[MAX_RX_LEN] = {0}; // globaali puskuri
volatile bool rx_new = false;               // uusi viesti tullut vai ei, ASETETAAN TRUE KUN UUSI VIESTI
static char rx_buffer[RX_BUFFER_SIZE];
static size_t rx_index = 0;
const uint32_t MORSE_FREQ_HZ = 600;    // käytä soveltuvaa taajuutta
volatile bool button2_pressed = false; // asetetaan BUTTON2 ISR:ssä
bool morseShown = false;               // onko morse viesti näytetty

// Tehtävien määrittelyt prototyyppinä
static void buzzer_task(void *arg);
static void print_task(void *arg);
static void btn_fxn(uint gpio, uint32_t eventMask);
static void usbTask(void *arg);
void imu_task(void *pvParameters);
void tud_cdc_rx_cb(uint8_t itf);

// ALLA PÄÄOHJELMA
// TÄMÄ OHJELMA TOTEUTTAA MORSE-KOODIN LÄHETTÄMISTÄ PICO W:LTÄ TIETOKONEELLE JA TAKAISIN

int main()
{

    // init_hat_sdk alustaa HATin laitteet
    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of hat is done.

    init_led();     // alustaa LEDin
    init_display(); // jos näyttää I2C:llä
    init_buzzer();  // alustaa summerin

    // Alusta painike ja LED ja rekisteröi vastaava keskeytys tarvittaessa.
    // Keskeytyskäsittelijä on määritelty alapuolella nimellä btn_fxn
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_pull_up(BUTTON1);

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_pull_up(BUTTON2);

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);

    TaskHandle_t hBuzzerTaskHandle = NULL;
    TaskHandle_t hPrintTask = NULL;
    TaskHandle_t hUsb = NULL;
    TaskHandle_t hIMUTask = NULL;

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(print_task,         // Task function
                                    "print",            // Name of the task
                                    DEFAULT_STACK_SIZE, // Stack size
                                    NULL,               // Arguments
                                    2,                  // Priority
                                    &hPrintTask);       // Task handle

    if (result != pdPASS)
    {
        usb_serial_print("Print task creation failed\n");
        return 0;
    }

    result = xTaskCreate(buzzer_task,         // Task function
                         "buzzer",            // Name
                         1024,                // Stack size
                         NULL,                // Args
                         2,                   // Priority
                         &hBuzzerTaskHandle); // Handle

    if (result != pdPASS)
    {
        usb_serial_print("Buzzer Task creation failed\n");
        return 0;
    }

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
#endif

    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB
    tusb_init();
    // Initialize helper library to write in CDC0)
    usb_serial_init();

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

// ---- Task running USB stack ----
static void usbTask(void *arg)
{
    (void)arg;
    while (1)
    {
        tud_task(); // With FreeRTOS wait for events
                    // Do not add vTaskDelay.
    }
}

/**
 * @brief Summeritehtävä ilmoituksia ja melodioita varten.
 *
 * Tämä FreeRTOS-tehtävä tarkkailee ohjelman tilaa (programState)
 * ja soittaa sopivia melodioita summerilla. Lisäksi se ohjaa LED:iä
 * ja näyttöä melodian aikana.
 *
 * Käyttäytyminen:
 * - Kun programState on MSG_RECEIVED tai MSG_PRINTED, soitetaan
 *   "Mission Impossible" -melodia LED- ja näyttöpalautteen kanssa.
 * - Kun programState on MORSE_MSG_SENT, soitetaan "Hyvät, Pahat ja Rumat" -melodia.
 * - Melodiaa ennen LED/ näyttötehtävät keskeytetään
 *   ja sen jälkeen programState päivitetään ja LED-/näyttötehtävät
 *   palautetaan toimintaan.
 *
 * @param arg Käyttämätön parametri, vaaditaan FreeRTOS-tehtävän prototyypin mukaan.
 */

static void buzzer_task(void *arg)
{
    (void)arg;

    // "Hyvät, pahat ja rumat" -teemamusiikin lyhyt intro
    const uint32_t notes[] = {440, 587, 440, 587, 440, 349, 392, 293};
    const uint32_t durs[] = {150, 150, 150, 150, 1200, 600, 600, 1800};
    const size_t count = sizeof(notes) / sizeof(notes[0]);
    // "HYVÄT, PAHAT JA RUMAT" -teemamusiikin lyhyt intro LOPPUU

    // "MISIION IMPOSSIBLE" -teemamusiikin lyhyt intro ALKUSOITTO
    const uint32_t notes_M_I[] = {196, 0, 196, 233, 261, 196, 0, 196, 175, 185};
    const uint32_t durs_M_I[] = {300, 150, 450, 300, 300, 300, 150, 450, 300, 300};
    const size_t count_M_I = sizeof(notes_M_I) / sizeof(notes_M_I[0]);
    // "MISSION IMPOSSIBLE" -teemamusiikin lyhyt intro ALKUSOITTO

    // "MISIION IMPOSSIBLE" -teemamusiikin lyhyt intro LOPPUSOITTO
    const uint32_t notes_M_I_2[] = {880, 698, 587, 880, 698, 554, 880, 698, 523, 466, 523};
    const uint32_t durs_M_I_2[] = {150, 150, 1500, 150, 150, 1500, 150, 150, 1500, 150, 150};
    const size_t count_M_I_2 = sizeof(notes_M_I_2) / sizeof(notes_M_I_2[0]);
    // "MISSION IMPOSSIBLE" -teemamusiikin lyhyt intro LOPPUSOITTO

    while (1)
    {
        // ---- MISSION IMPOSSIBLE TEEMA, KUN VIESTI SAAPUNUT/ TULOSTETTU ----
        if (programState == MSG_RECEIVED || programState == MSG_PRINTED)
        {

            clear_display(); // tyhjennetään näyttö

            set_led_status(true); // laitetaan led päälle myös MELODIAN ajaksi

            if (programState == MSG_RECEIVED)
            {
                write_text("VIESTI"); // näytetään merkki MELODIAN AJAKSI
                // Soita "MISSION IMPOSSIBLE" melodian alku
                for (int i = 0; i < 2; i++)
                {
                    for (size_t j = 0; j < count_M_I; j++)
                    {
                        if (notes_M_I[j] == 0)
                        {
                            sleep_ms(durs_M_I[j]); // tauko
                        }
                        else
                        {
                            buzzer_play_tone(notes_M_I[j], durs_M_I[j]);
                        }
                    }
                }
            }
            else if (programState == MSG_PRINTED)
            {
                write_text("OVER"); // näytetään merkki MELODIAN AJAKSI
                // Soita "MISSION IMPOSSIBLE" melodian alku
                for (size_t i = 0; i < count_M_I_2; i++)
                {
                    if (notes_M_I_2[i] == 0)
                    {
                        sleep_ms(durs_M_I_2[i]); // tauko
                    }
                    else
                    {
                        buzzer_play_tone(notes_M_I_2[i], durs_M_I_2[i]);
                    }
                }
            }

            set_led_status(false);

            clear_display(); // tyhjennä näyttö MELODIAN jälkeen

            // MELODIAN jälkeen vaihdetaan tila printattavaksi jos MSG_RECEIVED
            if (programState == MSG_RECEIVED)
                programState = MSG_PRINT;
            else
                programState = WAITING;
        }

        // Tarkkaile tilaa
        if (programState == MORSE_MSG_SENT)
        {

            morseShown = false;    // resetoi morseShown lippu seuraavaa keräämistä varten

            // ILMOITETAAN SERIAL CLIENTILLE ETTÄ VIESTI LOPPUI
            tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)"  \n", 3);
            tud_cdc_n_write_flush(CDC_ITF_TX);

            clear_display();         // tyhjennetään näyttö
            write_text(" MSG SENT"); // näytetään merkki MELODIAN AJAKSI

            // Soita "HYVÄT PAHAT JA RUMAT" melodian alku
            for (size_t i = 0; i < count; i++)
            {
                if (notes[i] == 0)
                {
                    sleep_ms(durs[i]); // tauko
                }
                else
                {
                    buzzer_play_tone(notes[i], durs[i]);
                }
            }

            // palautetaan tila lähtöön
            programState = WAITING;
            clear_display(); // tyhjennä näyttö MELODIAN jälkeen
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // pollataan tila 10 kertaa sekunnissa
    }
}

/**
 * @brief Viestin tulostustehtävä morsekoodina.
 *
 * Tämä FreeRTOS-tehtävä tarkistaa, onko saapunut uusi viesti (rx_new)
 * ja tulostaa sen merkkikerrallaan morsekoodina:
 * - Summeri ja LED vilkkuvat morsekoodin mukaan.
 * - Näyttö näyttää symbolin keskellä koko merkin ajan.
 * - Kun viesti on tulostettu, programState päivitetään MSG_PRINTED-tilaan.
 *
 * @param arg Käyttämätön parametri, vaaditaan FreeRTOS-tehtävän prototyypin mukaan.
 */

static void print_task(void *arg)
{
    (void)arg;

    const uint32_t unit_ms = 100;         // perusyksikkö morse-ajalle, haettu netistä nämä(säädä tarpeen mukaan)
    const uint32_t dot_ms = unit_ms;      // pisteen kesto
    const uint32_t dash_ms = 3 * unit_ms; // viivan kesto

    while (1)
    {

        // Jos uutta viestiä saapui (globaali lipuke asetettu ja tila MSG_PRINT)
        if (rx_new && programState == MSG_PRINT)
        {
            // Kopioi viesti paikalliseen puskuriin (turvallisuus)
            char local[MAX_RX_LEN];
            size_t len = strlen((const char *)rx_message);
            if (len >= MAX_RX_LEN)
                len = MAX_RX_LEN - 1;
            memcpy(local, (const char *)rx_message, len);
            local[len] = '\0';

            // Tyhjennä lipuke heti (estää uudelleenkäsittelyn)
            rx_new = false;

            // Tyhjennä näyttö
            clear_display();

            // Tulosta debugiin mitä soitetaan
            if (usb_serial_connected())
            {
                usb_serial_print("Playing received Morse on buzzer/LED:\n");
                usb_serial_print(local);
                usb_serial_flush();
            }

            // Toista merkkien mukaan morseaakkosena
            // local sisältää merkit: '.', '-', ' ' ja mahdollisesti '\n'
            for (size_t i = 0; local[i] != '\0'; ++i)
            {
                char c = local[i];

                clear_display();

                // Valmistele symboli
                char sym[3] = " ";
                if (c == '.')
                    sym[0] = '.';
                else if (c == '-')
                    sym[0] = '-';
                else if (c == ' ')
                    sym[0] = ' '; // välilyönti = tyhjä
                else
                {
                    sym[0] = c;
                } // jos jotain muuta

                /// Näytä symboli keskellä

                write_text(sym);

                /* Toista symboli (buzzer + LED) ja pidä näyttö siinä tilassa koko merkin ajan */
                if (c == '.')
                {
                    set_led_status(true);
                    buzzer_play_tone(MORSE_FREQ_HZ, dot_ms); // soittaa dot_ms aikaa
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(unit_ms));
                }
                else if (c == '-')
                {
                    set_led_status(true);
                    buzzer_play_tone(MORSE_FREQ_HZ, dash_ms); // soittaa dash_ms aikaa
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(unit_ms));
                }
                else if (c == ' ')
                {
                    /* välilyönti: kirjain-/sanaväli */
                    if (local[i + 1] == ' ')
                    {
                        /* sanaväli = 7 units */
                        vTaskDelay(pdMS_TO_TICKS(7 * unit_ms));
                        ++i; /* skippaa seuraavan välilyönnin, osa sanaväliä */
                    }
                    else
                    {
                        /* kirjainväli = 3 units */
                        vTaskDelay(pdMS_TO_TICKS(3 * unit_ms));
                    }
                }
                else if (c == '\n' || c == '\r')
                {
                    vTaskDelay(pdMS_TO_TICKS(3 * unit_ms));
                    break;
                }
                else
                {
                    /* tuntematon merkki esim ääkkönen -> pieni tauko */
                    vTaskDelay(pdMS_TO_TICKS(unit_ms));
                }
            }

            // Kun esitys valmis, vaihdetaan tila MSG_PRINTED
            programState = MSG_PRINTED;
            clear_display(); // tyhjennä näyttö lopuksi jotta viesti ei jää siihen näkyviin
        }

        // Kevyt odotus ennen seuraavaa tarkistusta
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---- Painikkeen keskeytyskäsittelijä ----
// Painiketta painettaessa vaihdetaan tila WAITING -> COLLECTING

static void btn_fxn(uint gpio, uint32_t events)
{
    if (gpio == BUTTON1 && (events & GPIO_IRQ_EDGE_FALL))
    {

        // Painnikkeen 1 havaittua muutetaan laitteen tilaa SWITCH rakenteen avulla

        switch (programState)
        {
        case WAITING:
            programState = COLLECTING;
            break;

        case COLLECTING:
            programState = MORSE_MSG_SENT;
            break;

        default:
            programState = WAITING;
            // muissa tiloissa nappi keskeyttää toiminnan ja palauttaa WAITING-tilaan
            break;
        }
    }

    if (gpio == BUTTON2 && (events & GPIO_IRQ_EDGE_FALL))
    {
        button2_pressed = true;
    }
}

// IMU TEHTÄVÄSSÄ OLLAAN KUN COLLECTING ON OHJELMAN TILANA

void imu_task(void *pvParameters)
{
    (void)pvParameters;
    clear_display();

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;
    char outbuf[128];

    // Alusta IMU kunnes onnistuu
    while (1)
    {
        int r = init_ICM42670();
        if (r == 0)
        {
            usb_serial_print("ICM-42670P initialized successfully!\n");
            if (ICM42670_start_with_default_values() != 0)
            {
                usb_serial_print("ICM42670_start_with_default_values returned non-zero\n");
            }
            break;
        }
        else
        {
            snprintf(outbuf, sizeof(outbuf), "ICM init failed (ret=%d), retrying in 2s\n", r);
            usb_serial_print(outbuf);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    enum
    {
        IDLE,
        DOT_STATE,
        DASH_STATE
    } motion_state = IDLE;

    // edelliset arvot delta-laskentaa varten
    float prev_ax = 0, prev_az = 1;

    while (1)
    {
        if (programState == COLLECTING)
        {
            if (!morseShown)
            {
                clear_display();
                write_text("MORSENOW");
                morseShown = true;
            }

            // --- Button2 käsittely: välilyönti ---
            if (button2_pressed)
            {
                char sym = ' ';
                clear_display();
                write_text("SPACE");
                sleep_ms(100);
                clear_display();

                // Lähetä heti CDC0:lle
                tud_cdc_n_write(CDC_ITF_TX, (uint8_t *)&sym, 1);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                usb_serial_print("Sent symbol: SPACE\n");

                button2_pressed = false;
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // --- IMU-luku ---
            int rr = ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
            if (rr != 0)
            {
                snprintf(outbuf, sizeof(outbuf), "IMU read failed (ret=%d)\n", rr);
                usb_serial_print(outbuf);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // --- Suodatettu liike ja delta ---
            float delta_ax = fabs(ax - prev_ax);
            float delta_az = fabs(az - prev_az);
            prev_ax = ax;
            prev_az = az;

            // --- DEBUG: tulosta JOKA KERTA ---
            snprintf(outbuf, sizeof(outbuf), "ax=%.3f  az=%.3f\n", prev_ax, prev_az);
            usb_serial_print(outbuf);

            // --- DOT tunnistus ---
            if (motion_state == IDLE && delta_az > 0.15f)
            {
                char sym = '.';
                clear_display();
                write_text(".");
                set_led_status(true);
                buzzer_play_tone(MORSE_FREQ_HZ, 100);
                set_led_status(false);
                clear_display();

                // Lähetä symboli välittömästi
                tud_cdc_n_write(CDC_ITF_TX, (uint8_t *)&sym, 1);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                usb_serial_print("Sent symbol: DOT\n");

                motion_state = DOT_STATE;
            }
            // --- DASH tunnistus ---
            else if (motion_state == IDLE && delta_ax > 0.15f)
            {
                char sym = '-';
                clear_display();
                write_text("-");
                set_led_status(true);
                buzzer_play_tone(MORSE_FREQ_HZ, 300);  
                set_led_status(false);
                clear_display();

                // Lähetä symboli välittömästi
                tud_cdc_n_write(CDC_ITF_TX, (uint8_t *)&sym, 1);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                usb_serial_print("Sent symbol: DASH\n");

                motion_state = DASH_STATE;
            }

            // --- Reset state kun liike loppuu ---
            if (motion_state != IDLE)
            {
                if ((motion_state == DOT_STATE && delta_az < 0.02f) ||
                    (motion_state == DASH_STATE && delta_ax < 0.01f))
                {
                    motion_state = IDLE;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20)); // pieni viive lukujen välillä
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief USB CDC -vastaanottokäsittelijä.
 *
 * Kutsutaan automaattisesti, kun dataa saapuu USB CDC -rajapintaan.
 * - Vastaanottaa tavut puskurille, kunnes rivinvaihto '\n' saapuu.
 * - Kopioi viestin globaaliin muuttujaan rx_message.
 * - Asettaa lipun rx_new merkiksi uudesta viestistä.
 * - Päivittää programState arvoksi MSG_RECEIVED.
 * - Lähettää takaisin "OK" -vastauksen USB:n yli.
 *
 * @param itf CDC-rajapinnan indeksi
 */

void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    if (count == 0)
        return;

    if (itf == CDC_ITF_TX)
    {
        for (uint32_t i = 0; i < count; i++)
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = buf[i];
            }

            // --- Viesti valmis kun rivinvaihto tulee ---
            if (buf[i] == '\n')
            {
                rx_buffer[rx_index] = 0; // null-terminate

                usb_serial_print("\nReceived on CDC 1: ");
                usb_serial_print(rx_buffer);

                // Kopioidaan viesti globaaliin muuttujaan print_taskille
                strncpy((char *)rx_message, rx_buffer, MAX_RX_LEN - 1);
                rx_message[MAX_RX_LEN - 1] = 0;

                // Asetetaan lipuke "uusi viesti tullut"
                rx_new = true;

                // Vaihdetaan tila vasta NYT kun viesti on valmis:
                programState = MSG_RECEIVED;

                // Vastataan OK vain kerran
                tud_cdc_n_write(itf, (uint8_t const *)"OK\n", 3);
                tud_cdc_n_write_flush(itf);

                // Nollataan vastaanottopuskuri seuraava viestiä varten
                rx_index = 0;
            }
        }
    }
}