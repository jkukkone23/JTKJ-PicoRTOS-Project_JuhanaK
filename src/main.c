#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"


#define DEFAULT_STACK_SIZE 2048

// Tilakoneen esittely lisää puuttuvat tilat tarvittaessa
enum state
{
    WAITING = 1,
    COLLECTING,
    SENT
};
enum state programState = WAITING;

// Tehtävien määrittelyt prototyyppinä
static void buzzer_task(void *arg);
static void display_task(void *arg);
static void leds_task(void *arg);
static void print_task(void *arg);
static void btn_fxn(uint gpio, uint32_t eventMask);

// ALLA PÄÄOHJELMA
// TÄMÄ OHJELMA TOTEUTTAA MORSE-KOODIN LÄHETTÄMISTÄ (USB-osio poistettu)

int main()
{
    // Alusta stdio (uart/usb MIEITI MITEN SAAT TOIMIMAAN
    stdio_init_all();

    // init_hat_sdk alustaa HATin laitteet
    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of hat is done.

    // Alusta painike ja LED ja rekisteröi vastaava keskeytys tarvittaessa.
    // Keskeytyskäsittelijä on määritelty alapuolella nimellä btn_fxn

    TaskHandle_t hLedsTaskHandle = NULL;
    TaskHandle_t hDisplayTaskHandle = NULL;
    TaskHandle_t hBuzzerTaskHandle = NULL;
    TaskHandle_t hPrintTask = NULL;

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(print_task,         // Task function
                                    "print",            // Name of the task
                                    DEFAULT_STACK_SIZE, // Stack size
                                    NULL,               // Arguments
                                    2,                  // Priority
                                    &hPrintTask);       // Task handle

    if (result != pdPASS)
    {
        printf("Print task creation failed\n");
        return 0;
    }

    result = xTaskCreate(leds_task,        // Task function
                         "led",            // Name
                         1024,             // Stack size
                         NULL,             // Args
                         2,                // Priority
                         &hLedsTaskHandle); // Handle

    if (result != pdPASS)
    {
        printf("Led Task creation failed\n");
        return 0;
    }

    result = xTaskCreate(buzzer_task,        // Task function
                         "buzzer",           // Name
                         1024,               // Stack size
                         NULL,               // Args
                         2,                  // Priority
                         &hBuzzerTaskHandle); // Handle

    if (result != pdPASS)
    {
        printf("Buzzer Task creation failed\n");
        return 0;
    }

    result = xTaskCreate(display_task,        // Task function
                         "display",           // Name
                         1024,                // Stack size
                         NULL,                // Args
                         2,                   // Priority
                         &hDisplayTaskHandle); // Handle

    if (result != pdPASS)
    {
        printf("Display Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

static void leds_task(void *arg)
{
    (void)arg;

    // Initialize LED (uses HAT SDK or pico functions inside)
    init_led();
    printf("Initializing on board led\n");

    while (1)
    {
        toggle_led();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void display_task(void *arg)
{
    (void)arg;

    init_display();
    printf("Initializing display\n");
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

static void buzzer_task(void *arg)
{
    (void)arg;

    init_buzzer();
    printf("Initializing buzzer\n");

    // "Hyvät, pahat ja rumat" -teemamusiikin lyhyt intro
    const uint32_t notes[] = { 440, 587, 440, 587, 440, 349, 392, 293 };
    const uint32_t durs[]  = { 150, 150, 150, 150, 900, 600, 600, 1200 };
    const size_t count = sizeof(notes) / sizeof(notes[0]);

    while (1)
    {
        // Tarkkaile tilaa
        if (programState == SENT)
        {
            // Soita melodian alku
            for (size_t i = 0; i < count; i++)
            {
                if (notes[i] == 0)
                {
                    sleep_ms(durs[i]);   // tauko
                }
                else
                {
                    buzzer_play_tone(notes[i], durs[i]);
                }
                sleep_ms(30); // pieni väli
            }

            // palautetaan tila lähtöön
            programState = WAITING;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // pollataan tila 10 kertaa sekunnissa
    }
}

static void btn_fxn(uint gpio, uint32_t eventMask)
{
    // Esimerkkitoiminto: vaihda LEDin tila
    (void)eventMask;
    // Jos käytät HAT-SDK:n funktiota, korvaa tarvittaessa:
    toggle_led();
}

static void print_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // Debug-tulostus: käytä printf/stdio (tulo UART/USB riippuu CMake-asetuksesta)
        printf("printTask\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
