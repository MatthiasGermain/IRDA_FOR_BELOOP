#include <Arduino.h>
#include "irda.h"
#include "oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h> 


#define BUF_SIZE 1024  // Réduction de la taille du buffer pour économiser de la mémoire

// Pin configuration for IRDA
#define IRDA_TX_PIN 32
#define IRDA_RX_PIN 35

// Définir l'intervalle d'envoi pour le radar
#define SEND_INTERVAL 3000  // Interval de temps pour envoyer un paquet IRDA

// Buffer for incoming data
char irda_buffer[BUF_SIZE];

#ifdef RECEIVER

// Définir la pwm pour la LED
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8  // Pour une valeur de 0 à 255

// LED Pin
#define LED_PIN 33
#define G_LED_PIN 25

const String CMD_GO = "GO";
const String CMD_SLOW = "SLOW";
const String CMD_STOP = "STOP";

// Vitesse de la capsule (analogique)
int VITESSE_CAPSULE = 0;
int VITESSE_CIBLE = 0;  // variable pour la vitesse cible

// Sémaphores pour la gestion des priorités
SemaphoreHandle_t oledMutex;  // Mutex pour l'accès à l'écran OLED
SemaphoreHandle_t prioritySemaphore; // Sémaphore pour gérer la priorité entre Serial et IRDA

// Déclaration globale des variables partagées
char last_message[BUF_SIZE] = ""; 
int last_speed = 0;  
String last_command = "";



// Fonction pour ajuster la vitesse PWM
void vitesse_pwm(int vitesse) {
    int pwm_value = map(vitesse, 0, 255, 0, 255);
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
                VITESSE_CIBLE = received_speed;
            } else {
                last_speed = 0;
            }

            strncpy(last_message, irda_buffer, BUF_SIZE);

            last_command = "IRDA";

            // Afficher sur l'OLED la dernière vitesse et la dernière consigne
            afficher_oled(last_speed, last_command);

            memset(irda_buffer, 0, BUF_SIZE);

            irda_flush();
        }
    // Libération du sémaphore pour les autres tâches
    xSemaphoreGive(prioritySemaphore);
    }
}

void handle_serial() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); 
        
        if (command != last_command) {
            if (command == "GO") {
                VITESSE_CIBLE = 255;
                last_command = CMD_GO;
                xSemaphoreGive(prioritySemaphore);
            } else if (command == "SLOW") {
                if (xSemaphoreTake(prioritySemaphore, 0) == pdTRUE) {
                    VITESSE_CIBLE = 64;
                    last_command = CMD_SLOW;
                } else {
                    VITESSE_CIBLE = 64;
                    last_command = CMD_SLOW;
                }
            } else if (command == "STOP") {
                if (xSemaphoreTake(prioritySemaphore, 0) == pdTRUE) {
                    VITESSE_CIBLE = 0;
                    last_command = CMD_STOP;
                } else {
                    VITESSE_CIBLE = 0;
                    last_command = CMD_STOP;
                }
            }
        }
    }
}

///////////////////////////////////////////////////////TACHES FREE RTOS///////////////////////////////////////////////////////


// Tâche FreeRTOS pour gérer la vitesse progressive
void task_vitesse(void *pvParameters) {
    while (1) {
        if (VITESSE_CAPSULE < VITESSE_CIBLE) {
            Serial.println("ACC Vitesse : " + String(VITESSE_CAPSULE));
            // Accélération progressive
            VITESSE_CAPSULE += 15;
            if (VITESSE_CAPSULE > VITESSE_CIBLE) VITESSE_CAPSULE = VITESSE_CIBLE;
        } 
        else if (VITESSE_CAPSULE > VITESSE_CIBLE) {
            Serial.println("DEC Vitesse : " + String(VITESSE_CAPSULE));
            // Décélération progressive
            int diff = VITESSE_CAPSULE - VITESSE_CIBLE;
            int deceleration = 15;  // Par défaut, même valeur que l'accélération

            // Appliquer la bonne formule de décélération selon la commande
            if (last_command == CMD_SLOW) {
                if (diff < 13) {
                    deceleration = -10 * log(1 - (diff / 13.0));
                } else {
                    deceleration = 30;
                }
            } 
            else if (last_command == CMD_STOP) {
                if (diff < 18) {
                    deceleration = -10 * log(1 - (diff / 18.0));
                } else {
                    deceleration = 50;
                }
            } 

            // Appliquer la décélération
            VITESSE_CAPSULE -= deceleration;
            if (VITESSE_CAPSULE < VITESSE_CIBLE) VITESSE_CAPSULE = VITESSE_CIBLE;
        }

        // Gestion de la G_LED
        if (VITESSE_CAPSULE != VITESSE_CIBLE) {
            digitalWrite(G_LED_PIN, HIGH);  // Allume la G_LED
        } else {
            digitalWrite(G_LED_PIN, LOW);   // Éteint la G_LED
        }

        // Mise à jour de la vitesse PWM
        vitesse_pwm(VITESSE_CAPSULE);
        afficher_oled(VITESSE_CAPSULE, last_command);

        vTaskDelay(pdMS_TO_TICKS(100));  // Mise à jour toutes les 100ms
    }
}


#endif

///////////////////////////////////////////////////////SETUP ET LOOP///////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);  // Initialisation du port série

    oled_init();  // Initialisation de l'écran OLED
    irda_init(IRDA_TX_PIN, IRDA_RX_PIN);  // Initialisation de la communication IRDA

    oled_display_text("Initialisation", 0, 0);

    #ifdef RECEIVER

    vitesse_pwm(VITESSE_CAPSULE);

    xTaskCreate(task_vitesse, "TaskVitesse", 2048, NULL, 1, NULL);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); 

    pinMode(G_LED_PIN, OUTPUT);
    digitalWrite(G_LED_PIN, LOW);

    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN, PWM_CHANNEL);
    
    // Création du mutex pour l'OLED
    oledMutex = xSemaphoreCreateMutex();

    // Création du sémaphore pour gérer la priorité (initialement donné)
    prioritySemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(prioritySemaphore);  // Par défaut, IRDA est autorisé

    #endif

}

void loop() {
#ifdef RADAR
    static unsigned long last_send_time = 0;

    if (millis() - last_send_time >= SEND_INTERVAL) {

        last_send_time = millis();

        oled_clear();

        int random_speed = random(0, 256); 
        char message[10];
        sprintf(message, "%d", random_speed);

        irda_send_packet(message);

        oled_display_text("Packet Sent", 0, 0);
        oled_display_text(message, 0, 16);

        delay(100);
    }
#endif

#ifdef RECEIVER
    handle_irda();   
    handle_serial(); 
#endif
}