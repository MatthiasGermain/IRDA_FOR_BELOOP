#include <Arduino.h>
#include "irda.h"
#include "oled.h"

#define BUF_SIZE 1024

// Pin configuration for IRDA
#define IRDA_TX_PIN 32
#define IRDA_RX_PIN 35

// LED Pin
#define LED_PIN 33

// Vitesse de la capsule (analogique)
int VITESSE_CAPSULE = 0; // De 0 à 255 pour une sortie analogique (PWM)

// Buffer for incoming data
char irda_buffer[BUF_SIZE];

// Mutex pour OLED
bool oled_busy = false;

// Intervalle d'envoi des paquets (en ms)
#define SEND_INTERVAL 5000

// Fonction pour ajuster la vitesse PWM
void vitesse_pwm(int vitesse) {
    // Map la vitesse entre 0 et 5 à une valeur de 0 à 255 pour PWM
    int pwm_value = map(vitesse, 0, 5, 0, 255);
    analogWrite(LED_PIN, pwm_value);  // Appliquer la vitesse via PWM
}

// Fonction d'affichage sur OLED avec verrouillage
void afficher_oled(int last_speed, String last_command) {

    // Variables pour suivre les dernières valeurs affichées
    static int last_displayed_speed;  // Dernière vitesse affichée
    static String last_displayed_command;  // Dernière commande affichée

    // Attendre que l'écran soit libre
    while (oled_busy) {
        delay(10);  // Attendre un peu avant de réessayer
    }

    // Verrouiller l'accès à l'écran OLED
    oled_busy = true;

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

    // Libérer l'accès à l'écran OLED
    oled_busy = false;
}

// Fonction pour gérer la réception IRDA
void handle_irda() {
    // Stocke le dernier message affiché pour éviter les doublons
    static char last_message[BUF_SIZE] = "";
    static int last_speed = 0;  // Dernière vitesse reçue
    static String last_command = "";  // Dernière consigne reçue (Serial)

    // Vérifier si l'OLED est disponible
    if (!oled_busy) {
        // Tenter de recevoir un paquet IRDA
        irda_receive_packet(irda_buffer, BUF_SIZE);

        // Si un message IRDA a été reçu et qu'il est différent du précédent
        if (strlen(irda_buffer) > 0 && strcmp(irda_buffer, last_message) != 0) {
            // Prendre le mutex
            oled_busy = true;

            // Vérifier si le message reçu est un chiffre valide entre 0 et 5
            int received_speed = atoi(irda_buffer);  // Convertir le message en entier

            // Afficher le message reçu dans le moniteur série
            Serial.print("Message reçu IRDA: ");
            Serial.println(received_speed);

            if (received_speed >= 0 && received_speed <= 5) {
                // Sauvegarder la dernière vitesse
                last_speed = received_speed;

                // Ajuster la vitesse via PWM
                vitesse_pwm(received_speed);
            } else {
                // Message invalide
                last_speed = 0;
                digitalWrite(LED_PIN, LOW); // LED éteinte si message invalide
            }

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

    // Afficher sur l'OLED la dernière vitesse et la dernière consigne seulement si elles ont changé
    afficher_oled(last_speed, last_command);
}

// Fonction pour gérer les commandes du port série
void handle_serial() {
    // Stocke la dernière commande reçue pour éviter les doublons
    static String last_command = "";

    // Lire les commandes du port série (GO, SLOW, STOP)
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Nettoyer la chaîne pour éviter les espaces inutiles

        if (command == "GO") {
            VITESSE_CAPSULE = 255; // Vitesse maximale
            last_command = "GO";
            vitesse_pwm(VITESSE_CAPSULE);  // Appliquer la vitesse via PWM
            Serial.println("GO Command Received");
        } else if (command == "SLOW") {
            VITESSE_CAPSULE = 64; // Vitesse réduite
            last_command = "SLOW";
            vitesse_pwm(VITESSE_CAPSULE);  // Appliquer la vitesse via PWM
            Serial.println("SLOW Command Received");
        } else if (command == "STOP") {
            VITESSE_CAPSULE = 0; // Arrêter la capsule
            last_command = "STOP";
            vitesse_pwm(VITESSE_CAPSULE);  // Appliquer la vitesse via PWM
            Serial.println("STOP Command Received");
        }
    }
}

void setup() {
    // Initialisation du moniteur série
    Serial.begin(115200);  // Initialisation du port série pour l'affichage dans le moniteur série

    // Initialisation de l'écran OLED
    oled_init();

    // Initialisation de la communication IRDA
    irda_init(IRDA_TX_PIN, IRDA_RX_PIN);

    // Initialisation de la LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // LED éteinte au départ

    // Initialisation de la vitesse PWM
    vitesse_pwm(VITESSE_CAPSULE);  // Initialiser la vitesse à 0

    // Affichage de l'état initial sur l'OLED
    oled_display_text("Initialisation", 0, 0);
}

void loop() {
#ifdef RADAR
    // Comportement de l'émetteur

    // Utilisation de millis() pour envoyer toutes les 5 secondes
    static unsigned long last_send_time = 0;

    // Vérifie si l'intervalle est écoulé
    if (millis() - last_send_time >= SEND_INTERVAL) {

        digitalWrite(LED_PIN, HIGH); // Allumer la LED

        last_send_time = millis(); // Mise à jour du dernier envoi

        // Nettoyer l'affichage pour éviter la persistance
        oled_clear();

        // Création du message avec une vitesse aléatoire entre 0 et 5
        int random_speed = random(0, 6); // Génère un nombre entre 0 et 5
        char message[10];
        sprintf(message, "%d", random_speed); // Message avec la vitesse

        // Envoi du paquet
        irda_send_packet(message);

        // Affichage de l'état sur l'OLED
        oled_display_text("Packet Sent", 0, 0);
        oled_display_text(message, 0, 16);

        delay(50);

        digitalWrite(LED_PIN, LOW);

        delay(100);
    }
#endif

#ifdef RECEIVER
    // Appels séparés pour plus de clarté
    handle_irda();   // Gérer la réception IRDA
    handle_serial(); // Gérer les commandes série
#endif
}
