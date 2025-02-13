#include <Arduino.h>
#include "irda.h"
#include "oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BUF_SIZE 1024  // Réduction de la taille du buffer pour économiser de la mémoire

// Pin configuration for IRDA
#define IRDA_TX_PIN 32
#define IRDA_RX_PIN 35

// Définir la pwm pour la LED
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8  // Pour une valeur de 0 à 255

// LED Pin
#define LED_PIN 33

// Définir l'intervalle d'envoi pour le radar
#define SEND_INTERVAL 3000  // Interval de temps pour envoyer un paquet IRDA

const String CMD_GO = "GO";
const String CMD_SLOW = "SLOW";
const String CMD_STOP = "STOP";

// Vitesse de la capsule (analogique)
int VITESSE_CAPSULE = 0;

// Buffer for incoming data
char irda_buffer[BUF_SIZE];

// Sémaphores pour la gestion des priorités
SemaphoreHandle_t oledMutex;  // Mutex pour l'accès à l'écran OLED
SemaphoreHandle_t prioritySemaphore; // Sémaphore pour gérer la priorité entre Serial et IRDA



// Déclaration globale des variables partagées
char last_message[BUF_SIZE] = ""; 
int last_speed = 0;  
String last_command = "";

// Fonction pour ajuster la vitesse PWM
void vitesse_pwm(int vitesse) {
    int pwm_value = map(vitesse, 0, 5, 0, 255);
    ledcWrite(PWM_CHANNEL, pwm_value);
}

// Fonction d'affichage sur OLED avec mutex
void afficher_oled(int last_speed, String last_command) {
    static int last_displayed_speed;  
    static String last_displayed_command;

    // Prendre le mutex pour accéder à l'OLED
    if (xSemaphoreTake(oledMutex, (TickType_t) 10) == pdTRUE) {
        
        // Mettre à jour l'écran seulement si la vitesse ou la commande a changé
        if (last_speed != last_displayed_speed || last_command != last_displayed_command) {
            // Clear l'écran pour éviter les doublons
            oled_clear();

            // Afficher la dernière vitesse reçue
            char display_message[30];
            sprintf(display_message, "Vitesse: %d", last_speed);
            oled_display_text(display_message, 0, 16);

            // Afficher la dernière consigne reçue via Serial
            oled_display_text("Consigne:", 0, 32);
            oled_display_text(last_command.c_str(), 0, 48);

            // Mettre à jour les dernières valeurs affichées
            last_displayed_speed = last_speed;
            last_displayed_command = last_command;
        }

        // Libérer le mutex après l'affichage
        xSemaphoreGive(oledMutex);
    }
}

// Fonction pour gérer la réception IRDA
void handle_irda() {
    if (xSemaphoreTake(prioritySemaphore, (TickType_t) 10) == pdTRUE) {
        // Tenter de recevoir un paquet IRDA
        irda_receive_packet(irda_buffer, BUF_SIZE);

        if (strlen(irda_buffer) > 0 && strcmp(irda_buffer, last_message) != 0) {
            int received_speed = atoi(irda_buffer);

            if (received_speed >= 0 && received_speed <= 255) {
                last_speed = received_speed;
                vitesse_pwm(received_speed);
            } else {
                last_speed = 0;
                digitalWrite(LED_PIN, LOW); 
            }

            strncpy(last_message, irda_buffer, BUF_SIZE);

            // Afficher sur l'OLED la dernière vitesse et la dernière consigne
            afficher_oled(last_speed, "IRDA");

            memset(irda_buffer, 0, BUF_SIZE);

            irda_flush();
        }
    // Libération du sémaphore pour les autres tâches
    xSemaphoreGive(prioritySemaphore);
    }
}

// Fonction pour gérer les commandes du port série
void handle_serial() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); 
        
        // Vérifie si la nouvelle commande est différente de la précédente
        if (command != last_command) {
            if (command == "GO") {

                VITESSE_CAPSULE = 255;
                last_command = CMD_GO;
                // Libère le sémaphore pour permettre la lecture IRDA
                xSemaphoreGive(prioritySemaphore);

            } else if (command == "SLOW") {

                VITESSE_CAPSULE = 64;
                last_command = CMD_SLOW;
                // Prend le sémaphore pour bloquer la lecture IRDA
                xSemaphoreTake(prioritySemaphore, portMAX_DELAY);

            } else if (command == "STOP") {

                VITESSE_CAPSULE = 0;
                last_command = CMD_STOP;

                // Prend le sémaphore pour bloquer la lecture IRDA
                xSemaphoreTake(prioritySemaphore, portMAX_DELAY);
            }
            vitesse_pwm(VITESSE_CAPSULE);
            afficher_oled(VITESSE_CAPSULE, last_command);    
        }
    }
}

void setup() {
    Serial.begin(115200);  // Initialisation du port série

    oled_init();  // Initialisation de l'écran OLED
    irda_init(IRDA_TX_PIN, IRDA_RX_PIN);  // Initialisation de la communication IRDA

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); 

    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN, PWM_CHANNEL);

    vitesse_pwm(VITESSE_CAPSULE);

    oled_display_text("Initialisation", 0, 0);

    // Création du mutex pour l'OLED
    oledMutex = xSemaphoreCreateMutex();

    // Création du sémaphore pour gérer la priorité (initialement donné)
    prioritySemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(prioritySemaphore);  // Par défaut, IRDA est autorisé

}

void loop() {
#ifdef RADAR
    static unsigned long last_send_time = 0;

    if (millis() - last_send_time >= SEND_INTERVAL) {
        digitalWrite(LED_PIN, HIGH);

        last_send_time = millis();

        oled_clear();

        int random_speed = random(0, 256); 
        char message[10];
        sprintf(message, "%d", random_speed);

        irda_send_packet(message);

        oled_display_text("Packet Sent", 0, 0);
        oled_display_text(message, 0, 16);

        delay(50);

        digitalWrite(LED_PIN, LOW);

        delay(100);
    }
#endif

#ifdef RECEIVER
    handle_irda();   
    handle_serial(); 
#endif
}
