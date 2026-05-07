#include "pogobot.h"
#include "time.h"
#include <math.h>

#define CLEAN
/** last update: 2026-02-12
 * Diffusion for size estimation in a robot swarm
 * Define PHOTO_START to start every robot by flashing a light
 * Define MOVE to enable movement (run and tumble)
 * Define SOURCE_INIT to automatically choose the source instead of manually
**/

#define CODENAME "HANDSHAKE V3 - BANSHEE"
#define BANSHEE_VER 1   // 0, 1 (mode hello when partner says hello), 2 (mode ack when partner says hello)

#define FQCY 60 // Hz

/**
 * PHOTO START
 * comment #define PHOTO_START ==> pogobot runs immediately
 */
// #define PHOTO_START
#define LIGHT_THRESHOLD 50

/**
 * MESSAGES AND HANDSHAKE PROTOCOL
 */
#define INFRARED_POWER 2 // 1,2,3
#define MAX_NB_OF_MSG 6 // max. number of messages per step which this robot can record

#define P_SEND 0.2 // probability to send a message

// msg sizes
#define HELLO_MSG_SIZE 30 //26 // number of bytes. MANDATORY: must be consistant with message_template
#define ACK_MSG_SIZE 26
#define OK_MSG_SIZE 14

// msg types
#define HELLO 0
#define ACK 1
#define OK 2

// timeouts
#define T_MAX 50
#define ACK_TIMEOUT T_MAX
#define OK_TIMEOUT T_MAX

/**
 * DIFFUSION
 */
#define KAPPA 0.9

/**
 * SOURCE INITIALIZATION (manual)
 */
#define robot_idA 39726	// 26: 2349     13: 22571   3: 21547	38: 38958   18: 32812   189: 36140  47: 31275   148: 39726  128: 18216
                        // 78: 24878    14: 6959	4: 19759    39: 29483   19: 27437   193: 38177  43: 31022   106: 22568

/**
 * MOVEMENT
 * comment #define MOVE ==> no movement
 */
// #define MOVE
#define MOTOR_POWER 350
#define MIN_RUN_TIMESTEPS FQCY
#define MAX_RUN_TIMESTEPS FQCY*3 
#define MIN_TUMBLE_TIMESTEPS FQCY/10 
#define MAX_TUMBLE_TIMESTEPS FQCY  
#define RUN 0
#define TUMBLE 1

#define DEBUG 2 // 0 without fick (100 exchanges), 1 without fick debug, 2 fick integration, 3 fick integration + debug

// nevermind
#define NB_MSG_TO_SEND 100

//for the probility

#pragma pack(push)
#pragma pack(1)

typedef struct RawMessage {
    int robot_id; // uint16_t
	uint8_t msg_type;   // HELLO, ACK or OK
    int recipient_id; // -1 if HELLO
    int hello_msg_id;
	uint8_t probe_attempt;
    int partner_msg_id;	// if ACK: specify the id of the msg we are acknowledging. -1 otherwise
    double concentration_value;
    int diffusion_ind;
} RawMessage;


typedef union message_template {
	uint8_t pogobot_message[HELLO_MSG_SIZE];
	RawMessage values;
} message;


typedef struct connecting_robot {
	int robot_id;
	int msg_id;
	int partner_msg_id;
    double concentration_value;
    int diffusion_ind;

} connecting_robot;

#pragma pack(pop)



#define PRECISION 10000 // conversion double<->int (ex.: 100 is 2 decimals) -- note: when sending/receiving, payload is a list of *bytes*
#define NB_DIGIT log_10(PRECISION)
void send_message(uint8_t msg_type, int hello_msg_id, int recipient_id, int partner_msg_id, int my_id, uint8_t probe_attempt, double concentration_value, int diffusion_ind);
int double_get_decimal_part ( double value );
int double_get_integer_part ( double value );
void double_get_decimal_part_string(double value, char *res);
void clear_connecting_neighbour(connecting_robot *robot);
int log_10( int power_of_ten );
int round_double(const double d);
void set_rgb_colors(double concentration);

//diffusion
int last_diffusion_seen = 0;
int diffusion_ind = 0;


// power_of_ten can only be 1, 10, 100, 1000... etc
int log_10(int power_of_ten){
    int pw = 0;
    int i = 1;
    while (i < power_of_ten){
        pw++;
        i*=10;
    }

    return pw;
}

int round_double(const double d) 
{ 
	return (int) (d + 0.5);
} 

int double_get_decimal_part ( double value )
{
	return (int)((value-(int)value)*PRECISION);
}



void double_get_decimal_part_string(double value, char *res){
    
    int dec_part = double_get_decimal_part(value);
    
    int j=10;
    res[NB_DIGIT] = '\0';
    for (int i=NB_DIGIT-1; i>=0 && j<=PRECISION; i--){
        res[i] = (char)((int)((dec_part%j)/(j/10))) + '0';
        j*=10;
    }
}

int double_get_integer_part ( double value )
{
	return (int)value;
}

void set_rgb_colors(double concentration){
	// concentration is between 0 and 1
	// goal: convert the concentration value into a rgb color
	// from 1 to 100
	// tens: HEAD 
	// units: BACK
	// 0: black, 10(0): blue, 9(0): baby blue, 8(0): cyan, 7(0): mint, 6(0): green, 5(0): yellow, 4(0): orange, 3(0): baby pink, 2(0): magenta, 1(0): red
	int blue[3] =       {0,0,25};
	int baby_blue[3] =  {6,6,25};
	int cyan[3] =       {0,25,25};
	int mint[3] = 		{6,25,6};
    int green[3] =      {0,25,0};
    int yellow[3] =     {12,6,0};
    int orange[3] =     {25,6,0};
    int baby_pink[3] = 	{12,3,12};
	int magenta[3] =    {25,0,25};
	int red[3] =        {25,0,0};
    

	int estimation = round_double(1/concentration);
	int r_unit, g_unit, b_unit, r_ten, g_ten, b_ten;

	// units
	int unit = estimation%10;
	if (unit == 0){
		r_unit = 0;
		g_unit = 0;
		b_unit = 0;
	} 
	if (unit == 1){
		r_unit = red[0];
		g_unit = red[1];
		b_unit = red[2];
	}
	if (unit == 2){
		r_unit = magenta[0];
		g_unit = magenta[1];
		b_unit = magenta[2];
	}
	if (unit == 3){
		r_unit = baby_pink[0];
		g_unit = baby_pink[1];
		b_unit = baby_pink[2];
	}
	if (unit == 4){
		r_unit = orange[0];
		g_unit = orange[1];
		b_unit = orange[2];
	}
	if (unit == 5){
		r_unit = yellow[0];
		g_unit = yellow[1];
		b_unit = yellow[2];
	}
	if (unit == 6){
		r_unit = green[0];
		g_unit = green[1];
		b_unit = green[2];
	}
	if (unit == 7){
		r_unit = mint[0];
		g_unit = mint[1];
		b_unit = mint[2];
	}
	if (unit == 8){
		r_unit = cyan[0];
		g_unit = cyan[1];
		b_unit = cyan[2];
	}
	if (unit == 9){
		r_unit = baby_blue[0];
		g_unit = baby_blue[1];
		b_unit = baby_blue[2];
	}

	// tens
	int ten = estimation - unit;
	if (ten == 0){
		r_ten = 0;
		g_ten = 0;
		b_ten = 0;
	} 
	if (ten == 10){
		r_ten = red[0];
		g_ten = red[1];
		b_ten = red[2];
	}
	if (ten == 20){
		r_ten = magenta[0];
		g_ten = magenta[1];
		b_ten = magenta[2];
	}
	if (ten == 30){
		r_ten = baby_pink[0];
		g_ten = baby_pink[1];
		b_ten = baby_pink[2];
	}
	if (ten == 40){
		r_ten = orange[0];
		g_ten = orange[1];
		b_ten = orange[2];
	}
	if (ten == 50){
		r_ten = yellow[0];
		g_ten = yellow[1];
		b_ten = yellow[2];
	}
	if (ten == 60){
		r_ten = green[0];
		g_ten = green[1];
		b_ten = green[2];
	}
	if (ten == 70){
		r_ten = mint[0];
		g_ten = mint[1];
		b_ten = mint[2];
	}
	if (ten == 80){
		r_ten = cyan[0];
		g_ten = cyan[1];
		b_ten = cyan[2];
	}
	if (ten == 90){
		r_ten = baby_blue[0];
		g_ten = baby_blue[1];
		b_ten = baby_blue[2];
	}
	if (estimation/100 >= 1) {
		r_ten = blue[0];
		g_ten = blue[1];
		b_ten = blue[2];
	}

	pogobot_led_setColors(r_ten, g_ten, b_ten, 3);
	pogobot_led_setColors(r_unit, g_unit, b_unit, 0);
}

void send_message(uint8_t msg_type, int hello_msg_id, int recipient_id, int partner_msg_id, int my_id, uint8_t probe_attempt, double concentration_value, int diffusion_ind)
{
	message my_message;
	int msg_size = HELLO_MSG_SIZE;
	if (msg_type == ACK)
		msg_size = ACK_MSG_SIZE;
	if (msg_type == OK)
		msg_size = OK_MSG_SIZE;


    my_message.values.robot_id = my_id;
	my_message.values.msg_type = msg_type;	// HELLO, ACK or OK
	my_message.values.recipient_id = recipient_id;
	my_message.values.hello_msg_id = hello_msg_id;
	my_message.values.partner_msg_id = partner_msg_id;
	my_message.values.probe_attempt = probe_attempt;

    my_message.values.concentration_value = concentration_value;
    my_message.values.diffusion_ind = diffusion_ind;


	uint8_t data[msg_size]; // message template	

	for ( int i = 0 ; i < msg_size ; i++ ){
		data[i] = my_message.pogobot_message[i];
	}

	pogobot_infrared_sendShortMessage_omni( (uint8_t *)( data ), msg_size );

	pogobot_led_setColors( rand()%25, rand()%25, rand()%25, 2);
}

void clear_connecting_neighbour(connecting_robot *robot){
	robot->robot_id = -1;
	robot->msg_id = 0;
    robot->partner_msg_id = 0;
    robot->concentration_value = 0.0;
    robot->diffusion_ind = 0;
}




// Define the global variables as previously discussed.

int main(void){
    int index_diff = 0; //comptage de diff
    double p; //proba de refaire une diff
    double t[5]; //tableau avec les différentes valeurs de concentrations sur les 10 dernieres diffusions
    double espsilon = 1e-9;
    // * init (mandatory)
    pogobot_init();
    srand( pogobot_helper_getRandSeed() );
    pogobot_infrared_set_power(INFRARED_POWER);


    /**
     * ROBOT GLOBAL DATA
     */
	int my_id = pogobot_helper_getid();

    // tick handling
    time_reference_t mystopwatch;
    int timestep = 0;

    // messages
    int hello_msg_id = 0;
    message received_message;

    // handshake
	connecting_robot my_partner;
    clear_connecting_neighbour(&(my_partner));

	int sending_HELLO = 1;
    int sending_ACK = 0;
    int sending_OK = 0;

    int msg_wait_step = 0;   // can be reset anytime

#ifndef CLEAN
    int sent_first_hello = 0;
    uint16_t one_handshake_iterations_firstHELLO = 0;
    uint16_t one_handshake_iterations_lastHELLO = 0;
    uint16_t one_handshake_iterations_receivedHELLO = 0;
	int other_robot_id = (my_id == robot_idA) ? robot_idB : robot_idA;

    int PROBE = 0;
    if (my_id == robot_idA || my_id == robot_idB)
        PROBE = 1;
    
    uint8_t probe_attempt = 0;
	int stop_listening = 0;
	int last_success = 0;
    double total_seconds = 0.0;
#endif
    int effective_msg_count = 0;

    // diffusion
    double concentration_value = 0.0;
    if (my_id == robot_idA)
        concentration_value = 1.0;
    
    // MOVEMENT
#ifdef MOVE
    int mov_timestep = 0;
    uint8_t dir[3];
    pogobot_motor_dir_mem_get(dir);   // RLB
    int moving_phase = RUN;
    int run_timesteps = rand()%(MAX_RUN_TIMESTEPS + 1 - MIN_RUN_TIMESTEPS) + MIN_RUN_TIMESTEPS;
    int tumble_timesteps = rand()%(MAX_TUMBLE_TIMESTEPS + 1 - MIN_TUMBLE_TIMESTEPS) + MIN_TUMBLE_TIMESTEPS;
#endif


    char dec_part[NB_DIGIT+1];

    if ( HELLO_MSG_SIZE != sizeof(received_message.values))
	{
		printf("[ERROR] !!!!!!!!!!!!!!!!!!!!!! HELLO_MSG_SIZE (%d) and message structure (%d) are NOT CONSISTANT !!!!\n", HELLO_MSG_SIZE, sizeof(received_message.values));
		printf("[ABORT]\n");
        pogobot_led_setColors(128,128,128,0); // error code: stopped.
		return -1;
	}


    int16_t last_data_b;
    int16_t last_data_fl;
    int16_t last_data_fr;
    /**
     * STARTING SIMULTANEOUSLY WITH FLASH LIGHT
    */
#ifdef PHOTO_START
    pogobot_led_setColor(0,0,8);

    last_data_b = pogobot_photosensors_read(0);
    last_data_fl = pogobot_photosensors_read(1);
    last_data_fr = pogobot_photosensors_read(2);

    while (1){

        /**
         * Read photo sensors
        */
        int16_t data_b = pogobot_photosensors_read(0);
        int16_t data_fl = pogobot_photosensors_read(1);
        int16_t data_fr = pogobot_photosensors_read(2);

        // stopping if the difference between the last value and the current value is more than the threshold
        int16_t avg_light_diff = ((data_b - last_data_b) + (data_fl - last_data_fl) + (data_fr - last_data_fr))/3;  // positive if data > last_data, i.e more light than before
        if (avg_light_diff >= LIGHT_THRESHOLD)
            break;


        last_data_b = data_b;
        last_data_fl = data_fl;
        last_data_fr = data_fr;
        
    }

    pogobot_led_setColor(0,0,0);

    msleep(5000);
#endif

    last_data_b = pogobot_photosensors_read(0);
    last_data_fl = pogobot_photosensors_read(1);
    last_data_fr = pogobot_photosensors_read(2);



    /**
     * 
     * MAIN LOOP
     * 
    */

    while (1) {

        pogobot_stopwatch_reset( &mystopwatch );
			
        // SHOTGUN STOP
        int16_t data_b = pogobot_photosensors_read(0);
        int16_t data_fl = pogobot_photosensors_read(1);
        int16_t data_fr = pogobot_photosensors_read(2);

        // stopping if the difference between the last value and the current value is more than the threshold
        int16_t avg_light_diff = ((data_b - last_data_b) + (data_fl - last_data_fl) + (data_fr - last_data_fr))/3;
        if (avg_light_diff >= LIGHT_THRESHOLD)
            break;


        last_data_b = data_b;
        last_data_fl = data_fl;
        last_data_fr = data_fr;

        // MOVE
#ifdef MOVE
        if (moving_phase == RUN) {
            if (mov_timestep >= run_timesteps){
                moving_phase = TUMBLE;
                tumble_timesteps = rand()%(MAX_TUMBLE_TIMESTEPS + 1 - MIN_TUMBLE_TIMESTEPS) + MIN_TUMBLE_TIMESTEPS;

                // on tire aleatoirement la direction
                int reverse_motor = rand()%2;
                pogobot_motor_dir_set(reverse_motor, 1-dir[0]);
                pogobot_motor_dir_set(1-reverse_motor, dir[1]);

                mov_timestep = 0;
            }
        } else {
            if (mov_timestep >= tumble_timesteps){
                moving_phase = RUN;
                run_timesteps = rand()%(MAX_RUN_TIMESTEPS + 1 - MIN_RUN_TIMESTEPS) + MIN_RUN_TIMESTEPS;

                // changing direction (backward/forward)
                dir[0] = 1 - dir[0];
                dir[1] = 1 - dir[1];
                // printf("%d %d\n", dir[0], dir[1]);
                pogobot_motor_dir_set(motorR, dir[0]);
                pogobot_motor_dir_set(motorL, dir[1]);

                mov_timestep = 0;
            }
        }

        pogobot_motor_power_set(motorL, MOTOR_POWER);
		pogobot_motor_power_set(motorR, MOTOR_POWER);
        mov_timestep++;
#endif

        int communication_established = 0;


        if (DEBUG > 1)
            set_rgb_colors(concentration_value);


        // *
        // * SENDING
        // *

        // *
        // * 3 SENDING STATES:
        // * - no handshaking robot => flooding a value. msg_type = HELLO, sending_HELLO = 1, my_partner.robot_id = -1
        // * - handshaking with a robot whom i have not sent a value to yet => sending ACK + my value back to my_partner.robot_id: sending_ACK = 1
        // * - handshaking with a robot whom i have not sent an OK to yet => sending an OK to my_partner.robot_id: sending_OK = 1
        // *

        int send = (rand()%100 <= P_SEND*100);  // percentage

        if (sending_HELLO){   
            if (send){
#ifndef CLEAN
                send_message(HELLO, -1, -1, my_id, probe_attempt, concentration_value, diffusion_ind);
#else
                send_message(HELLO, hello_msg_id, -1, -1, my_id, 0, concentration_value, diffusion_ind);
#endif

                if (DEBUG%2){
                    printf("[%d] sent HELLO %d\n", my_id, hello_msg_id);
                }
#ifndef CLEAN
                if (!sent_first_hello){
                    one_handshake_iterations_firstHELLO = 0;
                    sent_first_hello = 1;
                }
                one_handshake_iterations_lastHELLO = 0;
#endif
            }
        } 

        if (sending_ACK){   
            if (send){
#ifndef CLEAN
                send_message(ACK, hello_msg_id, my_partner.robot_id, my_partner.partner_msg_id, my_id, probe_attempt, concentration_value, diffusion_ind);
#else
                send_message(ACK, hello_msg_id, my_partner.robot_id, my_partner.partner_msg_id, my_id, 0, concentration_value, diffusion_ind);
#endif

                if (DEBUG%2){
                    printf("[%d] sent ACK to %d, %d\n", my_id, my_partner.robot_id, my_partner.partner_msg_id);
                }
            }
        }

        if (sending_OK){    
            if (send){
#ifndef CLEAN
                send_message(OK, my_partner.robot_id, -1, my_id, probe_attempt, concentration_value, diffusion_ind);
#else
                send_message(OK, hello_msg_id, my_partner.robot_id, -1, my_id, 0, concentration_value, diffusion_ind);
#endif

                if (DEBUG%2){
                    printf("[%d] sent OK %d, %d\n", my_id, my_partner.robot_id, hello_msg_id);
                }
            }
        }

        



        // *
        // * READING
        // *
        
        pogobot_infrared_update();

        if ( ( BANSHEE_VER > 0 ) || !(sending_OK) ) // read FIFO buffer - any message(s)?
        {
            int nb_msg = 0;

            // read messages. Some upper limits apply (first come first serve basis)
            while ( pogobot_infrared_message_available() && nb_msg < MAX_NB_OF_MSG )
            {

                message_t mr; // IMPORTANT: uses uint8 !!
                pogobot_infrared_recover_next_message( &mr );

                // ignoring incorrect messages
                if (mr.header.payload_length > HELLO_MSG_SIZE && mr.header.payload_length > ACK_MSG_SIZE && mr.header.payload_length > OK_MSG_SIZE)
                    continue;

                nb_msg = nb_msg + 1;

                for ( int i = 0 ; i != mr.header.payload_length ; i++ ) // load message
                    received_message.pogobot_message[i] = mr.payload[i];

                int sender_id = received_message.values.robot_id;

                // already establishing connection with someone else => ignoring
                if ((my_partner.robot_id != -1 && my_partner.robot_id != sender_id))
                    continue;


                if (DEBUG%2 && (received_message.values.recipient_id == my_id || received_message.values.recipient_id == -1)){
                    printf("[%d] received msg_type=%d, sender_id=%d, recipient_id=%d, hello_msg_id=%d, partner_msg_id=%d, probe_attempt=%d\n", my_id, received_message.values.msg_type, sender_id, received_message.values.recipient_id, received_message.values.hello_msg_id, received_message.values.partner_msg_id, received_message.values.probe_attempt);
                }



                /**
                 * 
                 * PROCESSING DEPENDING ON MSG TYPE
                 * 
                */
                // options:
                // 1) i receive a HELLO msg -> set my state to ACK 
                // 2) i receive an ACK -> check validity, if correct set my state to OK
                // 3) i receive an OK -> communication established

                // reading the messages only if i am interested
                if (sending_HELLO){
                    if (received_message.values.msg_type == HELLO){
                        // I RECEIVED A VALUE FROM THIS NEIGHBOUR, I WILL SEND BACK ACK

                        // ------------------- VALID MESSAGE -------------------

                        if (my_partner.robot_id == -1) {
                            my_partner.robot_id = sender_id;
#ifndef CLEAN                            
                            if (PROBE && sender_id == other_robot_id){
                                probe_attempt++;

                                if (received_message.values.probe_attempt > probe_attempt)
                                    probe_attempt = received_message.values.probe_attempt;

                            }
#endif
                        }
                        my_partner.partner_msg_id = received_message.values.hello_msg_id;
                        my_partner.concentration_value = received_message.values.concentration_value;
                        sending_ACK = 1;
                        sending_OK = 0;
                        sending_HELLO = 0;
                        msg_wait_step = 0;
                        // one_handshake_iterations_receivedHELLO++;

                        
                        break;

                    }

                    if (received_message.values.msg_type == ACK && received_message.values.recipient_id == my_id && received_message.values.partner_msg_id == hello_msg_id) {
                        // I RECEIVED AN ACK FROM THIS NEIGHBOUR, I WILL SEND BACK OK

                        // ------------------- VALID MESSAGE -------------------

                        // not establishing a connection with anyone yet => updating my_partner
#ifndef CLEAN
                        if (PROBE) {
                            if (sender_id == other_robot_id){
                                probe_attempt++;

                                if (received_message.values.probe_attempt > probe_attempt)
                                    probe_attempt = received_message.values.probe_attempt;
                            }
                        }
#endif
                        my_partner.robot_id = sender_id;
                        my_partner.concentration_value = received_message.values.concentration_value;
                        my_partner.partner_msg_id = received_message.values.hello_msg_id;
                        sending_OK = 1;
                        sending_HELLO = 0;
                        sending_ACK = 0;
                        msg_wait_step = 0;


                        break;
                    }
                }


                if (sending_ACK){
                    if (received_message.values.msg_type == OK && received_message.values.recipient_id == my_id && received_message.values.hello_msg_id == my_partner.partner_msg_id){
                        // I RECEIVED AN OK FROM THIS NEIGHBOUR 

                        // ------------------- VALID MESSAGE -------------------

                        communication_established = 1;
#ifndef CLEAN
                        if (PROBE) {
                            if (sender_id == other_robot_id){

                                if (received_message.values.probe_attempt > probe_attempt)
                                    probe_attempt = received_message.values.probe_attempt;

                            }
                        }
#endif

                        break;
                    } 

                    if (received_message.values.msg_type == ACK && sender_id == my_partner.robot_id && received_message.values.recipient_id == my_id && received_message.values.partner_msg_id == hello_msg_id) {
                        // I RECEIVED AN ACK FROM THIS NEIGHBOUR, I WILL SEND BACK OK

                        // ------------------- VALID MESSAGE -------------------
#ifndef CLEAN
                        if (PROBE) {
                            if (sender_id == other_robot_id){

                                if (received_message.values.probe_attempt > probe_attempt)
                                    probe_attempt = received_message.values.probe_attempt;

                            }
                        }
#endif                       
                        sending_OK = 1;
                        sending_HELLO = 0;
                        sending_ACK = 0;
                        msg_wait_step = 0;


                        break;
                    }
                    
                }


                if (sending_OK){
                    if (received_message.values.msg_type == HELLO){
                        // I RECEIVED A HELLO FROM THIS NEIGHBOUR: THEY FINISHED THE CONNECTION

                        // ------------------- VALID MESSAGE -------------------

                        communication_established = 1;

                        break;
                    } 
                }
                    
            }
        }

        pogobot_infrared_clear_message_queue();

        msg_wait_step++;


        if (sending_OK && msg_wait_step == OK_TIMEOUT){
            communication_established = 1;
            index_diff++;
        }


        // *
        // * TREATMENT
        // *

        int finish_connection = 0;
        int index = 0; 
        int step = 0; //combien de fois on a remplit le tableau complétement 
        int verif_eg = 0;
        if (communication_established){
            //double proba = (double) rand()/(double) RAND_MAX;
            
            t[index]=concentration_value;
            printf("T[%d] .%4f\n", index, t[index]);
            index++;
            index_diff++;
            if(index==5){
                verif_eg = 1; //valeur qui nous dira si toutes les valeurs du tableauw sont égaux
                for(int j=1;j<5;j++){
                    printf("T[0] .%4f\n", t[0]);
                    printf("T[%d] .%4f\n", j, t[j]);
                    if(fabs(t[j]-t[0])>=espsilon){
                        verif_eg = 0; 
                        break;
                    }
                }
                if(verif_eg == 1){
                    printf("Consensus, on peut relancer une redif\n");
                } //à intégrer dans la boucle
                index=0;
                step++;
            }
            if(step==1){
                step=0;
            }
        
        if(verif_eg==1){
            diffusion_ind++;
            printf("Relance diffusion %d (index %d)\n", diffusion_ind, index_diff);

            //printf("Je rentre dans le if %d\n", index_diff); //debug
            if (my_id == robot_idA){
                concentration_value=1.0;
                printf("C=1, %d\n", index_diff);
            }
            else{
                concentration_value=0.0;
                printf("C=0, %d\n", index_diff);
            }
            //index_diff=0;

            last_diffusion_seen = diffusion_ind;
            verif_eg = 0;

        }
            //si re diffusion on met tout le monde à jour
            else if (my_partner.diffusion_ind > last_diffusion_seen){

                printf("Nouvelle diffusion detectee %d\n", my_partner.diffusion_ind);

                diffusion_ind = my_partner.diffusion_ind;
                last_diffusion_seen = my_partner.diffusion_ind;

                if (my_id == robot_idA){
                    concentration_value = 1.0;
                }
                else{
                    concentration_value = 0.0;
                }
            }
            //sinon diffusion normale
            else{ /*--> faut voir pourquoi ça fonctionne pas*/ 
                printf("je rentre dans le else %d\n", index_diff); //debug
                printf("Robot %d | diff=%d\n", my_id, diffusion_ind);
                double delta_concentration = KAPPA * ( my_partner.concentration_value - concentration_value );
                concentration_value += delta_concentration;
                effective_msg_count++;
                double_get_decimal_part_string(concentration_value, dec_part);
            }
            if (DEBUG==3)
                printf("%d,%d,%d,%d,%d,%d.%s\n", my_id, timestep, my_partner.robot_id, hello_msg_id, my_partner.partner_msg_id, double_get_integer_part(concentration_value), dec_part);

#ifndef CLEAN
            if (PROBE && my_partner.robot_id == other_robot_id && !(stop_listening)){

                if (DEBUG%2)
                    printf("\033[32mPROBE attempt n°%d success\033[0m\n\n", probe_attempt);

                last_success = hello_msg_id;	// last time i received a msg from the other PROBE
                
                one_handshake_iterations_firstHELLO++;
                one_handshake_iterations_lastHELLO++;
                one_handshake_iterations_receivedHELLO++;
                total_seconds += 1e-6*pogobot_stopwatch_get_elapsed_microseconds( &(mystopwatch) );
                double_get_decimal_part_string(total_seconds, dec_part);

                if (sending_OK)
                    one_handshake_iterations_receivedHELLO = 0; // i am the one who sent hello and received ack
                else {
                    one_handshake_iterations_firstHELLO = 0;    // i was the one who received hello and sent ack
                    one_handshake_iterations_lastHELLO = 0;
                }

            }
    
#endif
            finish_connection = 1;
              
    } 
    else{

            if (sending_ACK && msg_wait_step == ACK_TIMEOUT) {
#ifndef CLEAN
                if (PROBE && my_partner.robot_id == other_robot_id && !(stop_listening)){
                    if (DEBUG%2)
                        printf("\033[31mPROBE attempt n°%d fail\033[0m\n\n", probe_attempt);
                }
#endif
                finish_connection = 1;
            }
        }


        if (finish_connection){
            hello_msg_id++;
#ifndef CLEAN
            one_handshake_iterations_firstHELLO = 0;
            one_handshake_iterations_lastHELLO = 0;
            one_handshake_iterations_receivedHELLO = 0;
            sent_first_hello = 0;
#endif
            if (BANSHEE_VER == 2 && sending_OK && msg_wait_step < OK_TIMEOUT){   // V1.2 -- detection of HELLO -> replying ACK
                sending_HELLO = 0;
                sending_ACK = 1;
                my_partner.partner_msg_id = received_message.values.hello_msg_id;
                my_partner.concentration_value = received_message.values.concentration_value;
            } else {
                clear_connecting_neighbour(&(my_partner));
                sending_HELLO = 1;
                sending_ACK = 0;
            }

            msg_wait_step = 0;
            sending_OK = 0;
        }




        double_get_decimal_part_string(concentration_value, dec_part);
        if (DEBUG==2)
            printf("%d,%d.%s,%d,%d\n", my_id, double_get_integer_part(concentration_value), dec_part, timestep, effective_msg_count);

        timestep++;


        // *
        // * Step synchronize - wait for next step (if not timed out already)
        // *

        uint32_t microseconds = pogobot_stopwatch_get_elapsed_microseconds( &(mystopwatch) );

        if (microseconds < 1000000 / FQCY)
        {
            pogobot_led_setColors(0,200,0,1);
            pogobot_led_setColors(0,0,0,1);
            msleep( (1000000 / FQCY - microseconds ) / 1000 ); // wait for next step.
        }
        else
        {
            pogobot_led_setColors(200,0,0,1);
            pogobot_led_setColors(0,0,0,1); 
            // too slow. Continue directly to next step.
        }
#ifndef CLEAN
        if (!(stop_listening)){
            one_handshake_iterations_firstHELLO++;
            one_handshake_iterations_lastHELLO++;
            one_handshake_iterations_receivedHELLO++;
            total_seconds += 1e-6*pogobot_stopwatch_get_elapsed_microseconds( &(mystopwatch) );
        }
#endif
    }

    // run terminated -- wait to ensure everyone stopped
    for (int i=0; i<5; i++){
        pogobot_led_setColors(0,0,0,i);
    }
    msleep(5000);

    pogobot_led_setColor(0,8,0);
    return 0;
}