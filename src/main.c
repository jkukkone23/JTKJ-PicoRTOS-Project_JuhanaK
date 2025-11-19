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
#define CDC_ITF_TX 1  // toinen CDC-rajapinta (debug/tkjhat)
#define RX_BUFFER_SIZE 128

//Tilakoneen esittely ---- lisää puuttuvat tilat tarvittaessa
//TILAT: WAITING, eli odotustila 
//COLLECTING, eli viestinkeräystila
//SENT, eli viestinlähetyksen jälkeinen tila, jolloinsoitetaan musiikki
enum state
{
    WAITING = 1,
    COLLECTING,
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
// TÄMÄ OHJELMA TOTEUTTAA MORSE-KOODIN LÄHETTÄMISTÄ 

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

    result = xTaskCreate(leds_task,        // Task function
                         "led",            // Name
                         1024,             // Stack size
                         NULL,             // Args
                         2,                // Priority
                         &hLedsTaskHandle); // Handle

    if (result != pdPASS)
    {
        usb_serial_print("Led Task creation failed\n");
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
        usb_serial_print("Buzzer Task creation failed\n");
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
    //Initialize helper library to write in CDC0)
    usb_serial_init();

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}

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

static void buzzer_task(void *arg)
{
    (void)arg;

    init_buzzer();
    usb_serial_print("Initializing buzzer\n");

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
        // --- CDC0: debug-viesti työasemaohjelmaan ---
        if (usb_serial_connected()) {
            usb_serial_print("Print task is running...\n");
            usb_serial_flush();
        }

        // --- CDC1: data toiseen CDC-rajapintaan ---
        if (tud_cdc_n_connected(CDC_ITF_TX)) {
            const char *msg = "Data sent over CDC1 interface, joka näkyy siis pythonissa\n";
            tud_cdc_n_write(CDC_ITF_TX, msg, strlen(msg));
            tud_cdc_n_write_flush(CDC_ITF_TX);
        }

        vTaskDelay(pdMS_TO_TICKS(4000)); // viive 4 sekuntia
    }
}

// callback when data is received on a CDC interface ELI TÄÄLLÄ SITTEN KÄSITELLÄÄN TULEVIA VIESTEJÄ KONEELTA
void tud_cdc_rx_cb(uint8_t itf){   
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    if (count == 0) return; // ei dataa -> ei käsittelyä

    if(itf == CDC_ITF_TX) {
        for(uint32_t i = 0; i < count; i++) {
            if(rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = buf[i];
            }

            // Viesti valmis rivinvaihdolla
            if(buf[i] == '\n') {
                rx_buffer[rx_index] = 0; // null-terminate
                usb_serial_print("\nReceived on CDC 1: ");
                usb_serial_print(rx_buffer);

                // Lähetä OK vain kerran per viesti
                tud_cdc_n_write(itf, (uint8_t const *) "OK\n", 3);
                tud_cdc_n_write_flush(itf);

                rx_index = 0; // nollaa bufferi seuraavaa viestiä varten
            }
        }
    }
}
