// Simple example receiving and sending MAVLink v2 over UDP
// based on POSIX APIs (e.g. Linux, BSD, macOS).
// https://github.com/BenbenIO/simple-Mavlink-C-rover/blob/master/rover.cpp

// todo: figure out mission and geofencing, going towards a certain direction
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <time.h>
#include <unistd.h>
#include <mavlink/ardupilotmega/mavlink.h>


void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_heartbeat(const mavlink_message_t* message);

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);

// void sleep(int seconds);
void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void set_guided_mode(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void arm(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void takeoff(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void start_mission(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);



int main(int argc, char* argv[])
{
    // Open UDP socket
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14550); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }
    printf("binded");
    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 100000;
    // if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    //     printf("setsockopt error: %s\n", strerror(errno));
    //     return -3;
    // }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    while (true) {
        // For illustration purposes we don't bother with threads or async here
        // and just interleave receiving and sending.
        // This only works  if receive_some returns every now and then.
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return 0;
}

void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    } 

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);

            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handle_heartbeat(&message);
                break;
            }
        }
    }
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("other");
            break;
    }
    printf(" autopilot\n");
}

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        // send_heartbeat(socket_fd, src_addr, src_addr_len);
        test_send_command(socket_fd, src_addr, src_addr_len);

        last_time = current_time;
    }
}

void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {
    set_guided_mode(socket_fd, src_addr, src_addr_len);
    arm(socket_fd, src_addr, src_addr_len);
    takeoff(socket_fd, src_addr, src_addr_len);
    sleep(10);
    start_mission(socket_fd, src_addr, src_addr_len);
}


void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;


    // [works] sending heartbeat
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);


    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}


// note that system = 1, target = 0. otherwise no response or ack from ardupilot
void set_guided_mode(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
    mavlink_command_long_t set_mode = {0};
     
    set_mode.target_system = 1; // must be 1 
	set_mode.target_component = 0; // must be 0
	set_mode.command = MAV_CMD_DO_SET_MODE;		// 176
	set_mode.confirmation = true;
	set_mode.param1 = 1; 				//need to be 1 ?? check			 	
	set_mode.param2 = 4; // 4 is GUIDED mode for drones (https://ardupilot.org/copter/docs/parameters.html#fltmode1)

    mavlink_msg_command_long_encode(1, 0, &message, &set_mode);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Set GUIDED Mode\n");
    }
}


void arm(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
     
    mavlink_command_long_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
	armed.command = MAV_CMD_COMPONENT_ARM_DISARM; //400
	armed.confirmation = true;
	armed.param1 = 1; // states (ready = 1)
	
	// Encode:
	mavlink_msg_command_long_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Drone is Armed\n");
    }
}

// note that parameter 7 determines the takeoff altitude
void takeoff(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_command_long_t takeoff = {0};
	takeoff.target_system = 1;
	takeoff.target_component = 0;
	takeoff.command = MAV_CMD_NAV_TAKEOFF; //400
	takeoff.confirmation = true;
	takeoff.param7 = 20; //takeoff altitude in meters
	
	// Encode:
	mavlink_msg_command_long_encode(1, 255, &message, &takeoff);



    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Drone is Taking Off\n");
    }
}


// note that mission can only be started using the mavlink_mission_item)int construct in ArduPilot
void start_mission(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {

    // go to waypoint: -35.313553, 149.162057

    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-waypoint
    // https://discuss.ardupilot.org/t/got-command-ack-nav-waypoint-unsupported/93054/2
    // https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/goto-location.py
	mavlink_mission_item_int_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
	mission.command = MAV_CMD_NAV_WAYPOINT; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; //MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    mission.current = 2,
    mission.autocontinue = 0,
	mission.param1 = 10;			 	//angle (centridegree) [-4500 - 4500]
	mission.param2 = 10;			 	//angle (centridegree) [-4500 - 4500]
	mission.param3 = 0;			 	//angle (centridegree) [-4500 - 4500]
	mission.param4 = 0;			 	//angle (centridegree) [-4500 - 4500]

	mission.x = (int) (-35.313553 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.y = (int) (149.162057 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.z = 20;	 			//speed normalized [0 - 1]
	
	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Mission Started! Waypoint = (-35.313553, 149.162057)\n");
    }
}

// void sleep(int seconds) {
//   usleep(seconds * 1000000);
// }
