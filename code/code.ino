
// iBoardBot PROJECT
// Author: Jose Julio & Juan pedro (JJROBOTS)
// Hardware: Arduino Leonardo + JJROBOTS BROBOT shield 2 (new) with ESP-12E Wifi module
// ESP8266 firmware version 1.0.1 or higher
// Date: 18/04/2015
// Last updated: 19/02/2017
// Version: 1.08
// Project page : http://jjrobots.com/
// Kickstarter page: https://www.kickstarter.com/projects/879074320/iboardbot-the-internet-controlled-whiteboard-robot
// GIT repository: https://github.com/jjrobots/iBoardbot
// License: Open Software GPL License v2

// Hardware:
// X Motor (longitudinal move) connected to MOTOR2 output
// Y Motor (vertical move) connected to MOTOR1 output
// Eraser servo connected to Servo1 output
// PEN Lift servo connected to Servo2 output

#include <EEPROM.h>
#include <avr/pgmspace.h>

#define VERSION "iBoardBot Cloudless 1.08.1"
#define DEBUG 1

// ROBOT and USER configuration parameters
#include "Configuration.h"

#define BUFFER_LEN 780
unsigned char buffer[BUFFER_LEN];   // buffer to store incomming message from server
bool draw_task = false;
bool new_packet = false;
int block_number;
bool next_block = false;


// Configuration: Pins, servos, Steppers, Wifi...
void setup()
{
    // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
    pinMode(4, OUTPUT); // ENABLE MOTORS
    pinMode(7, OUTPUT); // STEP MOTOR 1 PORTE,6
    pinMode(8, OUTPUT); // DIR MOTOR 1  PORTB,4
    pinMode(12, OUTPUT); // STEP MOTOR 2 PORTD,6
    pinMode(5, OUTPUT); // DIR MOTOR 2  PORTC,6
    pinMode(13, OUTPUT); // Servo pin
    digitalWrite(4, HIGH);  // Disable motors

    pinMode(10, OUTPUT);  // Servo1
    pinMode(6, OUTPUT);   // Servo2
    pinMode(2, OUTPUT);   // SERVOs aux (on I2C)

    delay(100);
    SERIAL_PORT.begin(115200); // serial output to console
    //while (!SERIAL_PORT);      // Arduino Leonardo wait for serial port to open...
    SERIAL_PORT.setTimeout(SERIAL_TIMEOUT_MS);

    delay(1000);

#ifdef DEBUG
    delay(10000);  // Only needed for serial debug
    SERIAL_PORT.println(VERSION);
    SERIAL_PORT.println(F("\nserial output prefixes:"));
    SERIAL_PORT.println(F("  CL - Cloudless data (from bot to host)"));
    SERIAL_PORT.println(F("  CD - Cloudless debug info"));
    SERIAL_PORT.println();
#endif

    // Init servos
    SERIAL_PORT.println(F("Servo init..."));
    servo_pos1 = SERVO1_ERASER;
    servo_pos2 = SERVO2_PAINT;
    initServo();
    moveServo1(SERVO1_ERASER);
    moveServo2(SERVO2_PAINT);
    delay(1000);
    disableServo1();
    disableServo2();

    //Initializing init position
    position_x = ROBOT_INITIAL_POSITION_X * X_AXIS_STEPS_PER_UNIT;
    position_y = ROBOT_INITIAL_POSITION_Y * Y_AXIS_STEPS_PER_UNIT;

    // Output parameters
    //SERIAL_PORT.print(F("Max_acceleration_x: "));
    //SERIAL_PORT.println(acceleration_x);
    //SERIAL_PORT.print(F("Max_acceleration_y: "));
    //SERIAL_PORT.println(acceleration_y);
    //SERIAL_PORT.print(F("Max speed X: "));
    //SERIAL_PORT.println(MAX_SPEED_X);
    //SERIAL_PORT.print(F("Max speed Y: "));
    //SERIAL_PORT.println(MAX_SPEED_Y);

    SERIAL_PORT.print(F("Free RAM: "));
    SERIAL_PORT.println(freeRam());

    // STEPPER MOTORS INITIALIZATION
    SERIAL_PORT.println(F("Steper motors initialization..."));
    // MOTOR1 = X axis => TIMER1
    TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
    OCR1A = MINIMUN_SPEED;               // Motor stopped
    dir_x = 0;
    TCNT1 = 0;

    // MOTOR2 = Y axis => TIMER3
    TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
    TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
    OCR3A = MINIMUN_SPEED;   // Motor stopped
    dir_y = 0;
    TCNT3 = 0;

    delay(200);
    //SERIAL_PORT.println("Moving to initial position...");
    // Initializing Robot command variables
    com_speed_x = MAX_SPEED_X / 2;
    com_speed_y = MAX_SPEED_Y / 2;
    max_speed_x = com_speed_x;
    max_speed_y = com_speed_y;

    //Initializing init position
    position_x = ROBOT_INITIAL_POSITION_X * X_AXIS_STEPS_PER_UNIT;
    position_y = ROBOT_INITIAL_POSITION_Y * Y_AXIS_STEPS_PER_UNIT;
    //last_position_x = position_x;
    //last_position_y = position_y;

    setSpeedS(com_speed_x, com_speed_y);
    setPosition_mm10(ROBOT_MIN_X * 10, ROBOT_MIN_Y * 10);
    last_move_x = ROBOT_INITIAL_POSITION_X * 10;
    last_move_y = ROBOT_INITIAL_POSITION_Y * 10;
    home_position = true;
    delay(50);

    // Enable TIMERs interrupts
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
    TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt

    delay(50);

    SERIAL_PORT.println(VERSION);
    //SERIAL_PORT.print("ID_IWBB ");
    SERIAL_PORT.println(" Ready...");
    timer_old = micros();
    timeout_counter = 0;
    block_number = -1;
    commands_index = 0;
    commands_lines = 0;
    draw_task = false;

    // Ready signal
    enableServo1();
    enableServo2();
    moveServo2(SERVO2_PAINT - 200);
    delay(300);
    moveServo2(SERVO2_PAINT);
    delay(300);
    moveServo2(SERVO2_PAINT - 200);
    delay(300);
    moveServo2(SERVO2_PAINT);
    delay(1000);

    disableServo1();
    disableServo2();
}


void loop()
{
    size_t bytes_read;
    char get_string[110];
    int code1;
    int code2;

    timer_value = micros();
    dt = timer_value - timer_old;
    if (dt >= 1000) { // 1Khz loop
        timer_old = timer_value;
        loop_counter++;
        if (draw_task) {
            adjustSpeed();
            positionControl(dt);   // position, speed and acceleration control of stepper motors
            timeout_counter++;
            wait_counter = 0;
            if ((timeout_counter > 8000) || ((myAbsLong(target_position_x - position_x) < POSITION_TOLERANCE_X) && (myAbsLong(target_position_y - position_y) < POSITION_TOLERANCE_Y))) { // Move done?
                if (timeout_counter > 8000) {
                    SERIAL_PORT.print(F("!TimeoutCounter! ")); // 8 seconds timeout
                    show_command = true;
                    // Reset position on timeout?
                }
#if DEBUG==2
                if (show_command) {
                    SERIAL_PORT.print(" EP:");
                    SERIAL_PORT.print(position_x);
                    SERIAL_PORT.print(":");
                    SERIAL_PORT.println(position_y);
                }
#endif
                if (commands_index < commands_lines) {
                    // Reading bytes from Packet (3 bytes per command)
                    int byte1 = buffer[commands_index * 3];
                    int byte2 = buffer[commands_index * 3 + 1];
                    int byte3 = buffer[commands_index * 3 + 2];
                    // Decode the command (code1 and code2) each code is 12 bits
                    code1 = (byte1 << 4) | ((byte2 & 0xF0) >> 4);
                    code2 = ((byte2 & 0x0F) << 8) | (byte3);

#if DEBUG==2
                    if (show_command) {
                        SERIAL_PORT.print(F("CD Command: "));
                        SERIAL_PORT.print(commands_index);
                        SERIAL_PORT.print(":");
                        SERIAL_PORT.print(code1);
                        SERIAL_PORT.print(",");
                        SERIAL_PORT.println(code2);
                        show_command = false;
                    }
#endif
                    // DECODE protocol codes
                    if ((new_packet) && (code1 != 4009)) { // check new packet
                        // PACKET ERROR: No valid first command!
                        SERIAL_PORT.print(F(" !PACKET ERROR!"));
                        commands_index = 0;
                        draw_task = false;
                        disableServo1();
                        disableServo2();
                        servo_counter = 0;
                        dir_x = 0;
                        dir_y = 0;
                    }
                    else if (code1 == 4009) {
                        new_packet = false;
                        block_number = code2;
                        if (block_number >= 4000)
                            block_number = -1;
                        else {
                            SERIAL_PORT.print(F("CD Start block: "));
                            SERIAL_PORT.println(block_number);
                        }
                        show_command = true;
                        servo_counter = 0;
                        if (timeout_recover) {   // Timeout recovery mode? This means we had a timeout
                            SERIAL_PORT.print(F("->Timeout recover!"));
                            // Rewrite a PEN LIFT COMMAND 4003 0
                            buffer[commands_index * 3] = (4003 >> 4);
                            buffer[commands_index * 3+1]= ((4003 << 4) & 0xF0);
                            buffer[commands_index * 3+2]= 0;
                            timeout_recover = false;
                        }
                        else
                            commands_index++;
                    }
                    else if ((code1 == 4001) && (code2 == 4001)) { // START DRAWING
                        if (servo_counter == 0) {
                            SERIAL_PORT.println(F(" !START DRAW"));
                            enableServo1();
                            enableServo2();
                        }
                        // Pen lift
                        moveServo1(SERVO1_PAINT);
                        moveServo2(SERVO2_LIFT);
                        servo_counter++;
                        if (servo_counter > 100) {
                            digitalWrite(4, LOW); // Enable motors...
                            // Default move speed
                            max_speed_x = MAX_SPEED_X;
                            max_speed_y = MAX_SPEED_Y;
                            erase_mode = 0;
                            commands_index++;
                            show_command = true;
                            timeout_counter = 0;
                            servo_counter = 0;
                        }
                    }
                    else if (code1 == 4002) { // END DRAWING
                        SERIAL_PORT.print(F(" !STOP DRAW time:"));
                        SERIAL_PORT.println(millis() - draw_init_time);
                        // Pen lift
                        if (!servo1_ready) {
                            enableServo1();
                            //SERIAL_PORT.println("EnableS1");
                        }
                        if (!servo2_ready) {
                            enableServo2();
                            //SERIAL_PORT.println("EnableS2");
                        }
                        moveServo1(SERVO1_ERASER);
                        moveServo2(SERVO2_PAINT);
                        digitalWrite(4, HIGH); // Disable motors...
                        dir_x = 0;
                        dir_y = 0;
                        erase_mode = 0;
                        draw_task = false;
                        commands_index = 0;
                        servo_counter = 0;
                        delay(300);    // Nothing to do ??
                        poll_again = true;
                        if (code2 == 4010) {  // Special code? => timeout_recovery on next block
                            timeout_recover = true;
                            SERIAL_PORT.print(F("->Timeout recover mode!"));
                        }
                        else
                            timeout_recover = false;
                        //stopServo();
                        disableServo1();
                        disableServo2();
                    }
                    else if (code1 == 4003) { // Pen lift command
                        if (!servo1_ready) {
                            enableServo1();
                        }
                        if (!servo2_ready) {
                            enableServo2();
                        }
                        moveServo1(SERVO1_PAINT);
                        moveServo2(SERVO2_LIFT);
                        servo_counter++;
                        if (servo_counter > 90) {
                            erase_mode = 0;
                            commands_index++;
                            show_command = true;
                            max_speed_x = MAX_SPEED_X;
                            max_speed_y = MAX_SPEED_Y;
                            timeout_counter = 0;
                            servo_counter = 0;
                            disableServo2();
                        }
                    }
                    // Pen down command
                    else if (code1 == 4004) {
                        if (!servo1_ready) {
                            enableServo1();
                        }
                        if (!servo2_ready) {
                            enableServo2();
                        }
                        moveServo1(SERVO1_PAINT);
                        moveServo2(SERVO2_PAINT);
                        servo_counter++;
                        //SERIAL_PORT.println(servo_pos);
                        if (servo_counter > 180) {
                            servo_pos2 = SERVO2_PAINT;
                            erase_mode = 0;
                            commands_index++;
                            show_command = true;
                            max_speed_x = SPEED_PAINT_X;
                            max_speed_y = SPEED_PAINT_Y;
                            timeout_counter = 0;
                            servo_counter = 0;
                            disableServo2();
                        }
                    }
                    // Eraser command
                    else if (code1 == 4005) {
                        if (!servo1_ready) {
                            enableServo1();
                        }
                        if (!servo2_ready) {
                            enableServo2();
                        }
                        // Make position correction for eraser
                        setPosition_mm10(last_move_x + ERASER_OFFSET_X * 10, last_move_y + ERASER_OFFSET_Y * 10);
                        moveServo1(SERVO1_ERASER);
                        moveServo2(SERVO2_PAINT);
                        servo_counter++;
                        //moveServo2(SERVO2_PAINT);
                        //SERIAL_PORT.println(servo_pos);
                        if (servo_counter > 350) {
                            servo_pos1 = SERVO1_ERASER;
                            erase_mode = 1;
                            commands_index++;
                            show_command = true;
                            max_speed_x = SPEED_ERASER_X;
                            max_speed_y = SPEED_ERASER_Y;
                            SERIAL_PORT.println(F(" SE"));
                            timeout_counter = 0;
                            servo_counter = 0;
                            disableServo1();
                            disableServo2();
                        }
                    }
                    // Wait command
                    else if (code1 == 4006) {
                        disableServo1();
                        disableServo2();
                        delay_counter++;
                        if (code2 > 30) // maximun 30 seconds of wait
                            code2 = 30;
                        if (delay_counter > ((long)code2 * 1000)) { // Wait for code2 seconds
                            SERIAL_PORT.println(F(" WM"));
                            commands_index++;
                            show_command = true;
                            delay_counter = 0;
                            timeout_counter = 0;
                        }
                    }
                    else {
                        if (servo1_ready)
                            disableServo1();
                        if (servo2_ready)
                            disableServo2();
                        timeout_counter = 0;
                        // Send coordinates to robot in mm/10 units
                        if (erase_mode == 1) { // In erase mode, we make the correction of the position of the eraser
                            setPosition_mm10(code1 + ERASER_OFFSET_X * 10, code2 + ERASER_OFFSET_Y * 10);
                            SERIAL_PORT.print("E");
                        }
                        else {
                            if ((code1 < 10) && (code2 < 10)) { // Home position?
                                setPosition_mm10(code1, code2);
                                home_position = true;
                            }
                            else {
                                setPosition_mm10(code1 + PAINT_OFFSET_X * 10, code2 + PAINT_OFFSET_Y * 10);
                                home_position = false;
                            }
                        }
                        last_move_x = code1;
                        last_move_y = code2;

                        commands_index++;
                        show_command = true;
                    }

                }
                else {
                    // End of commands...
                    SERIAL_PORT.print(F(" !FINISH! time:"));
                    SERIAL_PORT.println(millis() - draw_init_time);
                    commands_index = 0;
                    draw_task = false;
                    disableServo1();
                    disableServo2();
                    servo_counter = 0;
                    dir_x = 0;
                    dir_y = 0;
                }
            }

#if DEBUG==2
            if ((loop_counter % 50) == 0) {
                SERIAL_PORT.print(position_x);
                SERIAL_PORT.print(":");
                SERIAL_PORT.print(position_y);
                SERIAL_PORT.println();
            }
#endif
        }  // draw task
        else {
            // Polling server, task for me?
            loop_counter = 0;
            poll_again = false;
            if (!home_position) {
                if (wait_counter == 0)
                    wait_counter = millis();
                if ((millis() - wait_counter) > 8000) {
                    // Force Go to home command
                    SERIAL_PORT.println("->Force Home!");
                    draw_task = true;
                    new_packet = true;
                    digitalWrite(4, LOW); // Enable motors...
                    show_command = true;
                    timeout_counter = 0;
                    // ($code1>>4),(($code1<<4)&0xF0)|(($code2>>8)&0x0F),($code2&0xFF)
                    // 4009,4000
                    buffer[0] = (4009 >> 4);
                    buffer[1] = ((4009 << 4) & 0xF0) | ((4000 >> 8) & 0x0F);
                    buffer[2] = (4000 & 0xFF);
                    // 4001,4001
                    buffer[3] = (4001 >> 4);
                    buffer[4] = ((4001 << 4) & 0xF0) | ((4001 >> 8) & 0x0F);
                    buffer[5] = (4001 & 0xFF);
                    // 4003,0000
                    buffer[6] = (4003 >> 4);
                    buffer[7] = ((4003 << 4) & 0xF0) | ((0 >> 8) & 0x0F);
                    buffer[8] = 0;
                    // 1,1
                    buffer[9]  = 0;
                    buffer[10] = ((1 << 4) & 0xF0) | 0;
                    buffer[11] = 1;
                    // 4002,4002
                    buffer[12] = (4002 >> 4);
                    buffer[13] = ((4002 << 4) & 0xF0) | ((4010 >> 8) & 0x0F);  // Special END 4002 4010 => timeout recover
                    buffer[14] = (4010 & 0xFF);
                    commands_lines = 5;
                    draw_init_time = millis();
                    return; // End
                }
            } // if(!home_position)
            delay(20);
            SERIAL_PORT.println();
            SERIAL_PORT.println(F("CD Polling SERIAL_PORT..."));
            if (block_number == -1) {
                // Ready for new blocks...
                strcpy(get_string, "CL STATUS=READY");
                SERIAL_PORT.println(get_string);
                bytes_read = SERIAL_PORT.readBytes(buffer, BUFFER_LEN);
            } else {
                // ACK last block and ready for new one...
                strcpy(get_string, "CL STATUS=ACK&NUM=");
                char num[6];
                sprintf(num, "%d", block_number);
                strcat(get_string, num);
                SERIAL_PORT.println(get_string);
                bytes_read = SERIAL_PORT.readBytes(buffer, BUFFER_LEN);
            }
            SERIAL_PORT.print(F("CD Bytes read: "));
            SERIAL_PORT.println(bytes_read);
            if (bytes_read == 0) {
                SERIAL_PORT.println(F("!ERROR: M Timeout"));
            } else {
                if (bytes_read < 6) {
                    if (bytes_read > 0) {
                        SERIAL_PORT.print(F("Message:"));
                        SERIAL_PORT.print((char)buffer[0]);
                        SERIAL_PORT.println((char)buffer[1]);
                    }
                    block_number = -1;
                    // Error message?
                    if (buffer[0] == 'E') {
                        SERIAL_PORT.println("Error Message!");
                        SERIAL_PORT.println("Wait for next request...");
                        delay(10000);
                        // Something more here??
                    }
                    delay(100);
                } else {
                    SERIAL_PORT.print(F("CD Processing packet... Lines: "));
                    commands_lines = bytes_read / 3;
                    SERIAL_PORT.println(commands_lines);
                    commands_index = 0;
                    if (bytes_read == MAX_PACKET_SIZE) {
                        SERIAL_PORT.println(F("CD MORE BLOCKS!"));
                    }
                    draw_task = true;
                    new_packet = true;
                    digitalWrite(4, LOW); // Enable motors...
                    show_command = true;
                    timeout_counter = 0;
                    draw_init_time = millis();
                }
            } // if (bytes_read)
        } // draw task
    } // 1Khz loop
}
