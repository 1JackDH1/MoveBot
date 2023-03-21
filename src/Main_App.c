#include "xc.h"
#include "movement_functions.h"

int main(void) {
    FA_RobotInit();
    FA_LCDBacklight(30);
    FA_ResetEncoders();
    
    static robotState current_state = MLine_Driving_State;
    obstacle_pointer list_head = NULL;
    position robot_position = {.deviation_distance = 0, .angle = 0};
    double main_mline = 500, original_mline = 500; //written in mm

    while(current_state != Goal_State){
        current_state = hybrid_control(&main_mline, &robot_position, current_state, &list_head);
    }
    
    goal_functions(list_head, original_mline);
    return 0;
}
