# modified_br2_fsm_bumpgo_cpp

In this assignment, I tried my best to meet the requirements, however vague they were:
The appropriate changes were made to the header file by adding two new functions:
  bool check_forward_2_turn();
  bool is_front_clear();
  
For detecting obstacles on the right and left diagonal, under the "check_forward_2_turn" the ranges array was traversed to check if there is an obstacle in a broader range. If yes, the robot would go back for 2 seconds and turn.

Here is the snippet for the state code that changed: 
  
![Screenshot 2025-04-11 151715](https://github.com/user-attachments/assets/c234ec09-7f10-4e94-b8bc-4300a9b0aa67)


For the closed loop implementation:
  My robot would turn, without going back, until the range value at the front of the robot indicated there were no obstacles by checking the "is_front_clear()" method.

For the open loop implementation: 
  My robot would turn for a precalculated amount of time based on the angle which has the farthest away osbtacle (by obtaining the index of the max value in ranges array and converting to the corresponding angle) by using the formula x=vt -> t=x/v. thus, the robot turns until the time has elapsed. 
