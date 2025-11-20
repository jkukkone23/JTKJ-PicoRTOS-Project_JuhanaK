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
#define CDC_ITF_TX 1 // toinen CDC-rajapinta (debug/tkjhat)
#define RX_BUFFER_SIZE 128
#define MAX_RX_LEN 64

volatile char rx_message[MAX_RX_LEN] = {0}; // globaali puskuri
volatile bool rx_new = false;               // uusi viesti tullut vai ei, ASETETAAN TRUE KUN UUSI VIESTI

/* Globaalit tehtävä-handlet jotta voidaan suspend/resumeata eli keskeyttää/jatkaaniitä */
TaskHandle_t g_hDisplayTaskHandle = NULL;
TaskHandle_t g_hLedsTaskHandle = NULL;

// Tilakoneen esittely ---- lisää puuttuvat tilat tarvittaessa
// TILAT: WAITING, eli odotustila
// COLLECTING, eli viestinkeräystila Pico W:ltä
// MSG_RECEIVED, eli viesti vastaanotettu tila
// MSG_PRINT, eli viestin tulostustila (morse-koodina)
// MSG_PRINTED, eli viestin tulostuksen jälkeinen tila
// SENT, eli viestinlähetyksen jälkeinen tila, jolloinsoitetaan musiikki
enum state
{
    WAITING = 1,
    COLLECTING,
    MSG_RECEIVED,
    MSG_PRINT,
    MSG_PRINTED,
    SENT
};
enum state programState = WAITING;

static char rx_buffer[RX_BUFFER_SIZE];
static size_t rx_index = 0;

// Tehtävien määrittelyt prototyyppinä
static void buzzer_task(void *arg);
static void display_task(void *arg);
static void leds_task(void *arg);
static void print_task(void *arg);
static void btn_fxn(uint gpio, uint32_t eventMask);
static void usbTask(void *arg);
void tud_cdc_rx_cb(uint8_t itf);

// ALLA PÄÄOHJELMA
// TÄMÄ OHJELMA TOTEUTTAA MORSE-KOODIN LÄHETTÄMISTÄ PICO W:LTÄ TIETOKONEELLE JA TAKAISIN

int main()
{
    // Alusta stdio (uart/usb MIEITI MITEN SAAT TOIMIMAAN
    stdio_init_all();

    // init_hat_sdk alustaa HATin laitteet
    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of hat is done.

    // Alusta painike ja LED ja rekisteröi vastaava keskeytys tarvittaessa.
    // Keskeytyskäsittelijä on määritelty alapuolella nimellä btn_fxn

    TaskHandle_t hBuzzerTaskHandle = NULL;
    TaskHandle_t hPrintTask = NULL;
    TaskHandle_t hUsb = NULL;

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

    result = xTaskCreate(leds_task,           // Task function
                         "led",               // Name
                         1024,                // Stack size
                         NULL,                // Args
                         2,                   // Priority
                         &g_hLedsTaskHandle); // Handle

    if (result != pdPASS)
    {
        usb_serial_print("Led Task creation failed\n");
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

    result = xTaskCreate(display_task,           // Task function
                         "display",              // Name
                         1024,                   // Stack size
                         NULL,                   // Args
                         2,                      // Priority
                         &g_hDisplayTaskHandle); // Handle

    if (result != pdPASS)
    {
        usb_serial_print("Display Task creation failed\n");
        return 0;
    }

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
// ---- Task for led blinking ----
// Poista, jos tätä ei tarvita lopullisessa ohjelmassa
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
static void leds_task(void *arg)
{
    (void)arg;

    // Initialize LED (uses HAT SDK or pico functions inside)
    init_led();
    usb_serial_print("Initializing on board led\n");

    while (1)
    {
        toggle_led();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---- Task for display----
// Poista, jos tätä ei tarvita lopullisessa ohjelmassa
// MUISTA: ALUSTA DISPLAY JOSSAIN MUUALLA; JOS TÄTÄ EI KUTSUTA, EI NÄYTTÖÄ OLE ALUSTETTU
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA

static void display_task(void *arg)
{
    (void)arg;

    init_display();
    usb_serial_print("Initializing display\n");
    static int counter = 0;

    while (1)
    {
        clear_display();
        char buf[6]; // store up to 5 digits + null
        sprintf(buf, "%d", counter++);
        write_text(buf);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---- Task for buzzer----
//
// TÄLLÄ TEHTÄVÄLLÄ SOITETAAN MUSIIKKIA KUN VIESTI ON VASTAANOTETTU TAI LÄHETETTY
// HOITAA MYÖS MUUTAMAN TULOSTUKSEN SAMALLA; KUN VIESTI ON SAAPUNUT TAI LOPPUU
// SOITETAAN MISSION IMPOSSIBLE
// JOS VIESTI ON LÄHETETTY LAITTEELTA, SOITETAAN HYVÄT PAHAT JA RUMAT TEEMA
static void buzzer_task(void *arg)
{
    (void)arg;

    init_buzzer();
    usb_serial_print("Initializing buzzer\n");

    // "Hyvät, pahat ja rumat" -teemamusiikin lyhyt intro
    const uint32_t notes[] = {440, 587, 440, 587, 440, 349, 392, 293};
    const uint32_t durs[] = {150, 150, 150, 150, 1200, 600, 600, 1500};
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
            // ennen toistoa: keskeytä display ja leds taskit, jotta ne eivät piirrä tai välkytä
            if (g_hDisplayTaskHandle)
                vTaskSuspend(g_hDisplayTaskHandle);
            if (g_hLedsTaskHandle)
                vTaskSuspend(g_hLedsTaskHandle);

            clear_display(); // tyhjennetään näyttö

            set_led_status(true); // laitetaan led päälle myös MELODIAN ajaksi

            if (programState == MSG_RECEIVED)
            {
                write_text("VIESTI"); // näytetään merkki MELODIAN AJAKSI
                // Soita "MISSION IMPOSSIBLE" melodian alku
                for (size_t i = 0; i < count_M_I; i++)
                {
                    if (notes_M_I[i] == 0)
                    {
                        sleep_ms(durs_M_I[i]); // tauko
                    }
                    else
                    {
                        buzzer_play_tone(notes_M_I[i], durs_M_I[i]);
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

            // palautetaan display ja leds toimintaan
            if (g_hDisplayTaskHandle)
                vTaskResume(g_hDisplayTaskHandle);
            if (g_hLedsTaskHandle)
                vTaskResume(g_hLedsTaskHandle);
        }

        // Tarkkaile tilaa
        if (programState == SENT)
        {
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
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // pollataan tila 10 kertaa sekunnissa
    }
}

// ---- Task for Button----
// Poista, jos tätä ei tarvita lopullisessa ohjelmassa
// TÄTÄ EI KÄYNNISTETÄ VIELÄ PÄÄOHJELAMSSA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA
// POISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTAPOISTA

static void btn_fxn(uint gpio, uint32_t eventMask)
{
    // Esimerkkitoiminto: vaihda LEDin tila
    (void)eventMask;
    // Jos käytät HAT-SDK:n funktiota, korvaa tarvittaessa:
    toggle_led();
}

// ---- Task for buzzer----
//
// TÄMÄ TEHTÄVÄ TULOSTAA KONEELTA VASTAANOTETUN VIESTIN MORSE-KOODINA LAITTEELLE
// MERKKI KERRALLAAN JA SOITTAA SEN MYÖS SEKÄ SUMMERILLA ETTÄ LEDILLÄ

static void print_task(void *arg)
{
    (void)arg;

    const uint32_t unit_ms = 100;         // perusyksikkö morse-ajalle, haettu netistä nämä(säädä tarpeen mukaan)
    const uint32_t dot_ms = unit_ms;      // pisteen kesto
    const uint32_t dash_ms = 3 * unit_ms; // viivan kesto
    const uint32_t freq_hz = 600;         // buzzer-taajuus

    // Laitteiden alustukset
    init_buzzer();
    init_led();

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

            // ennen toistoa: keskeytä display ja leds taskit, jotta ne eivät piirrä tai välkytä
            if (g_hDisplayTaskHandle)
                vTaskSuspend(g_hDisplayTaskHandle);
            if (g_hLedsTaskHandle)
                vTaskSuspend(g_hLedsTaskHandle);

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
                    buzzer_play_tone(freq_hz, dot_ms); // soittaa dot_ms aikaa
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(unit_ms));
                }
                else if (c == '-')
                {
                    set_led_status(true);
                    buzzer_play_tone(freq_hz, dash_ms); // soittaa dash_ms aikaa
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
            // palautetaan display ja leds toimintaan
            if (g_hDisplayTaskHandle)
                vTaskResume(g_hDisplayTaskHandle);
            if (g_hLedsTaskHandle)
                vTaskResume(g_hLedsTaskHandle);

            // Kun esitys valmis, vaihdetaan tila MSG_PRINTED
            programState = MSG_PRINTED;
            clear_display(); // tyhjennä näyttö lopuksi jotta viesti ei jää siihen näkyviin
        }

        // Kevyt odotus ennen seuraavaa tarkistusta
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---- USB CDC RX callback ----
// TÄMÄ KUTSUTAAN KUN USB CDC RAJAPINTAAN TULEE DATAA
// ELI KUN KONEELTA ON LÄHETETTY VIESTI LAITTEELLE

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