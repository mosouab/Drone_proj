#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <math.h>

// Inclusion des headers MAVLink
// Chemin relatif depuis workspace/src -> workspace/libs
#include "../libs/mavlink/common/mavlink.h"

#define TARGET_IP "127.0.0.1"
#define TARGET_PORT 14580
#define BUFFER_LENGTH 2048

// Structure pour définir un waypoint
typedef struct {
    double lat_deg;
    double lon_deg;
    float alt_m;
} waypoint_t;

static int send_command_long(int sock,
                             const struct sockaddr_in *target_addr,
                             uint8_t target_system,
                             uint8_t target_component,
                             uint16_t command,
                             float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    mavlink_message_t cmd_msg;
    mavlink_command_long_t cmd;
    uint8_t tx_buf[BUFFER_LENGTH];

    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = target_system;
    cmd.target_component = target_component;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.param3 = p3;
    cmd.param4 = p4;
    cmd.param5 = p5;
    cmd.param6 = p6;
    cmd.param7 = p7;

    mavlink_msg_command_long_encode(255, 1, &cmd_msg, &cmd);
    int len = mavlink_msg_to_send_buffer(tx_buf, &cmd_msg);
    return (int)sendto(sock, tx_buf, len, 0, (const struct sockaddr *)target_addr, sizeof(*target_addr));
}

int main() {
    int sock;
    struct sockaddr_in targetAddr; 
    struct sockaddr_in locAddr;
    struct sockaddr_in fromAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;

    // Liste des waypoints à suivre
    // Coordonnées GPS pour SITL (région de Zurich par défaut)
    // Trajectoire en forme de cœur
    const waypoint_t waypoints[] = {
        {47.3975000, 8.5456000, 500.0f},  // Point bas du cœur
        {47.3976500, 8.5454000, 500.0f},  // Côté gauche montant
        {47.3978500, 8.5452500, 500.0f},  // Lobe gauche - bas
        {47.3980500, 8.5453000, 500.0f},  // Lobe gauche - milieu
        {47.3982000, 8.5454500, 500.0f},  // Lobe gauche - sommet
        {47.3982500, 8.5456000, 500.0f},  // Centre haut gauche
        {47.3982000, 8.5457500, 500.0f},  // Centre haut
        {47.3982500, 8.5459000, 500.0f},  // Centre haut droit
        {47.3982000, 8.5460500, 500.0f},  // Lobe droit - sommet
        {47.3980500, 8.5462000, 500.0f},  // Lobe droit - milieu
        {47.3978500, 8.5462500, 500.0f},  // Lobe droit - bas
        {47.3976500, 8.5461000, 500.0f},  // Côté droit descendant
        {47.3975000, 8.5459000, 500.0f},  // Retour vers le point bas
        {47.3975000, 8.5456000, 500.0f},  // Point bas du cœur (fermeture)
    };
    const int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

    // 1. Création du Socket UDP
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14540); // Notre port d'écoute (arbitraire)

    if (-1 == bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    } 

    // Configuration de l'adresse cible (PX4 SITL)
    memset(&targetAddr, 0, sizeof(targetAddr));
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_addr.s_addr = inet_addr(TARGET_IP);
    targetAddr.sin_port = htons(TARGET_PORT);

    printf("Attente du Heartbeat du drone...\n");

    // 2. Boucle principale
    int connection_established = 0;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    
    while(1) {
        memset(buf, 0, BUFFER_LENGTH);
        fromlen = sizeof(fromAddr);
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&fromAddr, &fromlen);
        
        if (recsize > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;

            // Parsing du paquet reçu
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);
                        printf("COMMAND_ACK: command=%u result=%u\n", ack.command, ack.result);
                    }
                    
                    // Si on reçoit un HEARTBEAT, la connexion est établie
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        if (!connection_established) {
                            printf("Drone connecté !\n");
                            connection_established = 1;

                            target_system = msg.sysid;
                            target_component = msg.compid;

                            // ÉTAPE A : ARMEMENT (Allumer les moteurs)
                            send_command_long(sock, &targetAddr, target_system, target_component,
                                              MAV_CMD_COMPONENT_ARM_DISARM,
                                              1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                            printf("Commande ARM envoyée.\n");

                            sleep(2); // Petite pause pour laisser le temps d'armer

                            // ÉTAPE B : DÉCOLLAGE (Takeoff)
                            send_command_long(sock, &targetAddr, target_system, target_component,
                                              MAV_CMD_NAV_TAKEOFF,
                                              0.0f, 0.0f, 0.0f, 0.0f,
                                              NAN, NAN, 500.0f);
                            printf("Commande TAKEOFF envoyée (50m).\n");

                            sleep(8); // Attendre que le drone décolle

                            // ÉTAPE C : NAVIGATION VERS WAYPOINTS
                            printf("Navigation vers %d waypoints...\n", num_waypoints);
                            for (int wp = 0; wp < num_waypoints; wp++) {
                                send_command_long(sock, &targetAddr, target_system, target_component,
                                                  MAV_CMD_DO_REPOSITION,
                                                  -1.0f,   // Ground speed (-1 = default)
                                                  1.0f,    // Bitmask (1 = use heading)
                                                  0.0f,    // Reserved
                                                  NAN,     // Yaw (NaN = maintain current)
                                                  waypoints[wp].lat_deg,
                                                  waypoints[wp].lon_deg,
                                                  waypoints[wp].alt_m);
                                printf("Waypoint %d/%d envoyé (lat=%.6f, lon=%.6f, alt=%.1fm)\n", 
                                       wp+1, num_waypoints,
                                       waypoints[wp].lat_deg, 
                                       waypoints[wp].lon_deg,
                                       waypoints[wp].alt_m);
                                
                                sleep(5); // Attendre entre chaque waypoint
                            }

                            // ÉTAPE D : ATTERRISSAGE (Landing)
                            printf("Navigation terminée. Commande LAND envoyée.\n");
                            send_command_long(sock, &targetAddr, target_system, target_component,
                                              MAV_CMD_NAV_LAND,
                                              0.0f, 0.0f, 0.0f, 0.0f,
                                              NAN, NAN, 0.0f);
                        }
                    }
                }
            }
        }
    }
    return 0;
}
