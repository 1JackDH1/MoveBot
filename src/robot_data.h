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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "allcode_api.h"
const int LEFT_TURN = -90;
const int RIGHT_TURN = 90;

typedef struct obstacle_info * obstacle_pointer;

typedef struct obstacle_info{
    int obstacle_num;
    double distance;
    obstacle_pointer next_obstacle;
} obstacle;

typedef struct position_data{
    double deviation_distance;
    int angle;
} position;

/*
 * Function to alter the robot's deviation distance
 * from the M-line.
 * robot_pos - pointer to structure for robot's positioning data
 * distance_marker - encoder value of the last placed distance marker
 * 
 * Determines the encoder distance using the marker and changes
 * the robot's deviation based on its current angle of orientation.
 */
void change_deviation_distance(position * robot_pos, double distance_marker){
    double value = fabs(FA_ReadEncoder(0) - distance_marker);
    if(robot_pos->angle == RIGHT_TURN){
        robot_pos->deviation_distance += value;
    }
    else if(robot_pos->angle == LEFT_TURN){
        robot_pos->deviation_distance -= value;
    }
}

/*
 * Function to create a new obstacle structure
 * and allocate dynamic memory for it.
 * num - obstacle number
 * distance - distance the robot has travelled around the obstacle
 * 
 * Returns a pointer to the new obstacle.
 */
obstacle_pointer make_obstacle(int num, double distance){
    obstacle_pointer new_obstacle_ptr = malloc(sizeof(obstacle));
    if(new_obstacle_ptr == NULL){
        return NULL;
    }
    else{
        new_obstacle_ptr->obstacle_num = num;
        new_obstacle_ptr->distance = distance;
        return new_obstacle_ptr;
    }
}

/*
 * Function to insert an obstacle to the head of the list.
 * head_ptr - pointer of a pointer of the head of the list
 * new_obstacle_ptr - pointer to the new obstacle
 */
void head_insert(obstacle_pointer * head_ptr, obstacle_pointer new_obstacle_ptr){
    new_obstacle_ptr->next_obstacle = *head_ptr;
    *head_ptr = new_obstacle_ptr;
}

/*
 * Function to the print out all the necessary data.
 * obstacle_list - pointer to the head of the list
 * mline - distance value of the original M-line
 */
void print_data(obstacle_pointer obstacle_list, double mline){
    obstacle_pointer next = obstacle_list;
    double extended_distance = 0;
    char num_detected[50];
    char line_distance[50];
    char actual_distance[50];
    char obstacle[50];
    char obstacles[50];
    sprintf(num_detected, "Obstacles: %d", next->obstacle_num);
    sprintf(line_distance, "M-Line: %.0fmm", mline);
    while(next != NULL){
        extended_distance += next->distance;
        sprintf(obstacles, "Obs-%d: %.0f", next->obstacle_num, next->distance);
        strcat(obstacles, obstacle);
        strcpy(obstacle, obstacles);
        next = next->next_obstacle;
    }
    sprintf(actual_distance, "Actual: %.0fmm", extended_distance + mline);
    FA_LCDBacklight(100);
    FA_LCDPrint(num_detected, 50, 0, 0, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDPrint(line_distance, 50, 0, 8, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDPrint(obstacles, 50, 0, 16, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDPrint(actual_distance, 50, 0, 24, FONT_NORMAL, LCD_OPAQUE);
}

/*
 * Function to release the dynamically allocated memory.
 * obstacle_list - pointer of a pointer of the head of the list
 */
void free_linkedList(obstacle_pointer * obstacle_list){
    obstacle_pointer current = *obstacle_list;
    obstacle_pointer next;
    while(current != NULL){
        next = current->next_obstacle;
        free(current);
        current = next;
    }
    *obstacle_list = NULL;
}