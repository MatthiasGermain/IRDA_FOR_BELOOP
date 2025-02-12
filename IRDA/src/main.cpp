#include <Arduino.h>
#include "irda.h"
#include "oled.h"

#define BUF_SIZE 1024

// Pin configuration for IRDA
#define IRDA_TX_PIN 32
#define IRDA_RX_PIN 35

// LED Pin
#define LED_PIN 33

// Buffer for incoming data
char irda_buffer[BUF_SIZE];

// Mutex for OLED
bool oled_busy = false;

// Intervalle d'envoi des paquets (en ms)
#define SEND_INTERVAL 5000

void setup() {
    // Initialisation de l'écran OLED
    oled_init();

    // Initialisation de la communication IRDA
    irda_init(IRDA_TX_PIN, IRDA_RX_PIN);

    // Initialisation de la LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // LED éteinte au départ

    #ifdef RECEIVER
    oled_display_text("Receiver waiting for data", 0, 16);
    #endif

}

void loop() {

#ifdef RADAR
    // Comportement de l'émetteur

    // Utilisation de millis() pour envoyer toutes les 5 secondes
    static unsigned long last_send_time = 0;
    static int packet_count = 1; // Compteur de paquets

    // Vérifie si l'intervalle est écoulé
    if (millis() - last_send_time >= SEND_INTERVAL) {
        last_send_time = millis(); // Mise à jour du dernier envoi

        // Nettoyer l'affichage pour éviter la persistance
        oled_clear();
        
        digitalWrite(LED_PIN, HIGH);

        // Création du message
        char message[30];
        sprintf(message, "Hello IRDA #%d", packet_count);

        // Envoi du paquet
        irda_send_packet(message);

        // Affichage de l'état sur l'OLED
        oled_display_text("Packet Sent", 0, 0);
        oled_display_text(message, 0, 16);

        // Éteindre la LED après l'envoi
        digitalWrite(LED_PIN, LOW);

        // Délai court pour éviter un envoi multiple involontaire
        delay(100);

        // Incrémenter le compteur de paquets
        packet_count++;
    }
#endif

#ifdef RECEIVER
    // Comportement du récepteur

    // Stocke le dernier message affiché pour éviter les doublons
    static char last_message[BUF_SIZE] = "";

    // Vérifier si l'OLED est disponible
    if (!oled_busy) {
        // Tenter de recevoir un paquet
        irda_receive_packet(irda_buffer, BUF_SIZE);

        // Si un message a été reçu et qu'il est différent du précédent
        if (strlen(irda_buffer) > 0 && strcmp(irda_buffer, last_message) != 0) {
            // Prendre le mutex
            oled_busy = true;

            // Clear l'écran pour éviter les doublons
            oled_clear();

            // Afficher le message reçu sur l'OLED
            oled_display_text(irda_buffer, 0, 16);

            // Allumer la LED pendant 1 seconde
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);

            // Sauvegarder le dernier message affiché
            strncpy(last_message, irda_buffer, BUF_SIZE);

            // Vider le irda_buffer
            memset(irda_buffer, 0, BUF_SIZE);

            // Libérer le mutex
            oled_busy = false;

            // Flush le buffer UART seulement après avoir traité le message
            irda_flush();
        }
    }
#endif
}
