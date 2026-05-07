#include "pogobot.h"
#include "time.h"

/** last update: 2026-01-15
 * Diffusion for size estimation in a robot swarm
 * 
 * Code to collect data through an external robot
**/

// template-level constant

#define CODENAME "PROBING DIFFUSION"

#define FQCY 60


#define PHOTO_START 1
#define LIGHT_THRESHOLD 70

/**
 * MESSAGES
 */
#define INFRARED_POWER 2 // 1,2,3
#define HELLO_MSG_SIZE 26 
// msg types
#define HELLO 0
#define ACK 1
#define OK 2

#define PRECISION 10000 // conversion double<->int (ex.: 100 is 2 decimals) -- note: when sending/receiving, payload is a list of *bytes*
#define NB_DIGIT log_10(PRECISION)



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
} RawMessage;

#pragma pack(pop)

typedef union message_template {
	uint8_t pogobot_message[HELLO_MSG_SIZE];
	RawMessage values;
} message;


int double_get_decimal_part ( double value );
int double_get_integer_part ( double value );
int log_10( int power_of_ten );
void double_get_decimal_part_string(double value, char *res);


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



int main(void) {

    // * init (mandatory)
    pogobot_init();
    srand( pogobot_helper_getRandSeed() );
    pogobot_infrared_set_power(INFRARED_POWER);

	message my_message;
	if ( HELLO_MSG_SIZE != sizeof(my_message.values))
	{
		printf("[ERROR] !!!!!!!!!!!!!!!!!!!!!! HELLO_MSG_SIZE and message structure are NOT CONSISTANT !!!!\n");
		printf("[ABORT]\n");
        pogobot_led_setColors(128,128,128,0); // error code: stopped.
		return -1;
	}

	char dec_part[NB_DIGIT+1];

	
	/**
	 * 
	 * STARTING SIMULTANEOUSLY
	 * 
	*/
	if (PHOTO_START){
        pogobot_led_setColor(0,0,8);
        int16_t last_data_b = pogobot_photosensors_read(0);
        int16_t last_data_fl = pogobot_photosensors_read(1);
        int16_t last_data_fr = pogobot_photosensors_read(2);

		while (1){

			/**
			 * Read photo sensors
			*/
			int16_t data_b = pogobot_photosensors_read(0);
			int16_t data_fl = pogobot_photosensors_read(1);
			int16_t data_fr = pogobot_photosensors_read(2);

			// stopping if the difference between the last value and the current value is more than the threshold
			int16_t diff_b = data_b - last_data_b;  // positive if data > last_data, i.e more light than before
			int16_t diff_fl = data_fl - last_data_fl;
			int16_t diff_fr = data_fr - last_data_fr;
			if (diff_b >= LIGHT_THRESHOLD || diff_fl >= LIGHT_THRESHOLD || diff_fr >= LIGHT_THRESHOLD)
				break;

			last_data_b = data_b;
			last_data_fl = data_fl;
			last_data_fr = data_fr;
			
		}
	}
    pogobot_led_setColor(0,0,0);

	time_reference_t mystopwatch;
	int timestep = 0;


    int16_t last_data_b = pogobot_photosensors_read(0);
    int16_t last_data_fl = pogobot_photosensors_read(1);
    int16_t last_data_fr = pogobot_photosensors_read(2);

	while (1) {
	    
        pogobot_stopwatch_reset(&mystopwatch);

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


		pogobot_infrared_update();

		if ( pogobot_infrared_message_available() ) // read FIFO buffer - any message(s)?
		{
			// read messages. Some upper limits apply (first come first serve basis)
			while ( pogobot_infrared_message_available())
			{
				message_t mr;
				pogobot_infrared_recover_next_message( &mr );

				// ignoring incorrect messages
				// if (mr.header.payload_length != HELLO_MSG_SIZE)
				// 	continue;

				for ( int i = 0 ; i != mr.header.payload_length ; i++ ) // load message
					my_message.pogobot_message[i] = mr.payload[i];

                if (my_message.values.msg_type == OK)
                    continue;

				int sender_id = my_message.values.robot_id;
				
				double concentration_value = my_message.values.concentration_value;

				pogobot_stopwatch_reset(&mystopwatch);
				double_get_decimal_part_string(concentration_value, dec_part);
				printf("%d,%d.%s,%d\n", sender_id, double_get_integer_part(concentration_value), dec_part, timestep);
			}

		}
		pogobot_infrared_clear_message_queue();


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

	}

    // run terminated -- wait to ensure everyone stopped
    for (int i=0; i<5; i++){
        pogobot_led_setColors(0,0,0,i);
    }
    
    pogobot_motor_power_set(motorR, 0);
    pogobot_motor_power_set(motorL, 0);
    msleep(5000);

    pogobot_led_setColor(0,8,0);
    

    return 0;
}
