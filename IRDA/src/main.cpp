#include <Arduino.h>
#include "irda.h"
#include "oled.h"

#define BUF_SIZE 1024

// Pin configuration for IRDA
#define IRDA_TX_PIN 32
#define IRDA_RX_PIN 35

// Buffer for incoming data
char irda_buffer[BUF_SIZE];

void setup() {
    Serial.begin(115200);

    // Initialize OLED display
    oled_init();

    // Initialize IRDA communication
    irda_init(IRDA_TX_PIN, IRDA_RX_PIN);

    // Display startup message
    oled_display_text("IRDA Ready", 0, 0);
}

void loop() {

#ifdef RADAR
    // Comportement de l'émetteur
    irda_send_packet("Hello IRDA");

    // Display a status message
    oled_display_text("Packet Sent", 0, 16);
    //Display beside the message send
    oled_display_text("Hello IRDA", 0, 32);

    delay(2000); // Wait for 2 seconds
#endif

#ifdef RECEIVER
    // Comportement du récepteur
    irda_receive_packet(irda_buffer, BUF_SIZE);

    if (strlen(irda_buffer) > 0) {
        //Clear the display
        oled_clear();

        // Afficher le message reçu sur l'OLED
        oled_display_text(irda_buffer, 0, 16);

        // Attendre une seconde avant de nettoyer l'écran
        delay(1000);
        oled_clear();
        //vider le irad_buffer
        memset(irda_buffer, 0, BUF_SIZE);
    }
#endif
}