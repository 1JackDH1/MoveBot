// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

#include "robot_data.h"
const int LEFT_MOTOR_STRAIGHT = 12;
const int RIGHT_MOTOR_STRAIGHT = 12;
const double ENCODER_TICK_CONVERT = 0.32;

/*
 * Enumeration of states within this program.
 */
typedef enum {
    MLine_Driving_State,
    Obstacle_Follow_State,
    Goal_State,
} robotState;

/*
 * Function for driving on the M-line.
 * distance - distance of M-line in millimetres
 * 
 * Makes comparisons with encoder readings and
 * only switches to the goal state after surpassing
 * the M-line distance.
 * 
 * Return the state of the robot based on behaviours
 */
robotState drive_MLine(double distance){
    robotState state = MLine_Driving_State;
    double value = fabs(FA_ReadEncoder(0) * ENCODER_TICK_CONVERT);
    FA_SetMotors(LEFT_MOTOR_STRAIGHT, RIGHT_MOTOR_STRAIGHT);
    if(value >= distance){
        FA_SetMotors(0,0);
        state = Goal_State;
    }
    return state;
}

/*
 * Function to realign the robot back onto
 * the M-line after following an obstacle.
 */
void mline_realignment(position * robot_pos){
    FA_Right(100);
    robot_pos->angle += RIGHT_TURN;
}

/*
 * Function primarily for obstacle following.
 * robot_pos - pointer to structure for robot's positioning data
 * current_state - current state of the robot
 * list_head - pointer to the pointer of the head of the obstacle list
 * mline - pointer to the current distance value of the M-line
 * 
 * Updates robot position data:
 *      Angle of orientation by constant values and occurrences of turning
 *      Deviation distance by encoder readings
 * Updates the current obstacle distance by encoder reading the start and end
 * Function has five states:
 *      1 - Initial wall alignment
 *      2 - Main following state
 *      3 - Collision and deviation prevention
 *      4 - Proceeding left turn after outward corner
 *      5 - Proceeding right turn after inward corner
 * 
 * Returns the state of the robot based on behaviours.
 */
robotState obstacle_following(position * robot_pos, robotState current_state, 
        obstacle_pointer * list_head, double * mline){
    static int follow_state = 1;
    static double distance_marker = 0;
    static double north_distance = 0;
    robotState state = current_state;
    
    switch(follow_state){
        case 1:
            FA_Right(40);
            if(FA_ReadIR(0) > 200){
                distance_marker = FA_ReadEncoder(0) * 5;
                robot_pos->angle = RIGHT_TURN;
                follow_state = 2;
            }
            break;
        case 2:
            if(robot_pos->angle == LEFT_TURN){
                if(fabs(FA_ReadEncoder(0) - distance_marker) >= robot_pos->deviation_distance){
                    FA_SetMotors(0,0);
                    (*list_head)->distance = (FA_ReadEncoder(0) * ENCODER_TICK_CONVERT) - 
                            (*list_head)->distance;
                    FA_PlayNote(200, 100);
                    *mline = fabs(*mline - north_distance);
                    state = MLine_Driving_State;
                    follow_state = 1;
                    mline_realignment(robot_pos);
                    break;
                }
            }
            FA_SetMotors(LEFT_MOTOR_STRAIGHT, RIGHT_MOTOR_STRAIGHT + 1);
            //detect outward corner
            if(FA_ReadIR(0) < 100){
                change_deviation_distance(robot_pos, distance_marker);
                follow_state = 4;
                break;
            }
            //detect inward corner
            else if(FA_ReadIR(2) > 200){
                change_deviation_distance(robot_pos, distance_marker);
                follow_state = 5;
                break;
            }
            follow_state = 3;
            break;
        case 3:
            if(FA_ReadIR(1) > 150){
                FA_SetMotors(LEFT_MOTOR_STRAIGHT + 10, RIGHT_MOTOR_STRAIGHT);
            }
            else if(FA_ReadIR(7) > 150){
                FA_SetMotors(LEFT_MOTOR_STRAIGHT, RIGHT_MOTOR_STRAIGHT + 10);
            }
            else{
                follow_state = 2;
            }
            break;
        case 4:
            FA_SetMotors(LEFT_MOTOR_STRAIGHT, RIGHT_MOTOR_STRAIGHT + 70);
            if(FA_ReadIR(1) > 400){
                FA_SetMotors(0,0);
                robot_pos->angle += LEFT_TURN;
                FA_Right(30);
                if(robot_pos->angle == 0){
                    north_distance += fabs((FA_ReadEncoder(0) * ENCODER_TICK_CONVERT) - north_distance);
                }
                distance_marker = FA_ReadEncoder(0);
                follow_state = 2;
            }
            break;
        case 5:
            FA_SetMotors(LEFT_MOTOR_STRAIGHT, RIGHT_MOTOR_STRAIGHT * -1);
            if(FA_ReadIR(2) < 20){
                FA_SetMotors(0,0);
                robot_pos->angle += RIGHT_TURN;
                FA_Right(40);
                if(robot_pos->angle == 0){
                    north_distance += fabs((FA_ReadEncoder(0) * ENCODER_TICK_CONVERT) - north_distance);
                }
                distance_marker = FA_ReadEncoder(0);
                follow_state = 3;
            }
            break;
        default:
            break;
    }
    return state;
}

/*
 * Function for controlling combination of behaviours.
 * mline - pointer to the current distance value of the M-line
 * robot_pos - pointer to structure for robot's positioning data
 * current_state - current state of the robot
 * list - pointer to the pointer of the head of the obstacle list
 * 
 * Allows for the robot to switch between the states and creates
 * new obstacle structures when encountered.
 * 
 * Return the state of the robot.
 */
robotState hybrid_control(double * mline, position * robot_pos, 
        robotState current_state, obstacle_pointer * list){
    static int obstacle_num = 0;
    robotState state = current_state;
    
    switch(state){
        case MLine_Driving_State:
            state = drive_MLine(*mline);
            if(FA_ReadIR(2) > 500){
                FA_SetMotors(0,0);
                FA_PlayNote(141, 100);
                robot_pos->deviation_distance = 0;
                obstacle_pointer new_obs = make_obstacle(++obstacle_num, 
                        FA_ReadEncoder(0) * ENCODER_TICK_CONVERT);
                head_insert(list, new_obs);
                state = Obstacle_Follow_State;
            }
            break;
        case Obstacle_Follow_State:
            state = obstacle_following(robot_pos, state, list, mline);
            break;
        case Goal_State:
            FA_SetMotors(0,0);
            state = Goal_State;
            break;
        default:
            break;
    }
    return state;
}

void play_goal_tune(){
    FA_PlayNote(233, 50);
    FA_PlayNote(261, 50);
    FA_PlayNote(277, 30);
    FA_PlayNote(277, 30);
    FA_PlayNote(311, 50);
    FA_PlayNote(261, 80);
    FA_PlayNote(233, 10);
    FA_PlayNote(207, 150);
    FA_PlayNote(233, 30);
    FA_PlayNote(233, 30);
    FA_PlayNote(261, 50);
    FA_PlayNote(277, 50);
    FA_PlayNote(233, 50);
    FA_PlayNote(207, 50);
    FA_PlayNote(415, 100);
    FA_PlayNote(415, 50);
    FA_PlayNote(311, 100);
}

/*
 * Function consisting of other function relevant to the goal state.
 * list_head - pointer to the head of the list
 * mline - distance value of the original M-line
 */
void goal_functions(obstacle_pointer list_head, double mline){
    print_data(list_head, mline);
    free_linkedList(&list_head);
    play_goal_tune();
}