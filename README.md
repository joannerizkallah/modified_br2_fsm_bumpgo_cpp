# modified_br2_fsm_bumpgo_cpp

In this assignment, I tried my best to meet the requirements, however vague they were:
The appropriate changes were made to the header file and .cpp file.
  
1. For detecting obstacles on the right and left diagonal, under the "check_forward_2_turn" the ranges array was traversed to check if there is an obstacle in a broader range. If yes, the robot would go back for 2 seconds and turn.

Here is the snippet for the FORWARD state code that changed: 
  
![Screenshot 2025-04-11 151715](https://github.com/user-attachments/assets/c234ec09-7f10-4e94-b8bc-4300a9b0aa67)

The check_forward_2_back() was updated to:

![image](https://github.com/user-attachments/assets/a072548f-122e-4ea7-a737-3f15e397d030)
Where the robot checks if there obstacles between the range of -pi/4 and pi/4, and goes back if true.

The turn state in this case was also updated:

![image](https://github.com/user-attachments/assets/f302cb67-2b08-408a-a662-e7e5c8e27024)

The robot will turn according to a specified direction variable turn_direction_ and will check if the time to turn to that specified direction of either -pi/4 or pi/4 was satisfied using the check_turnleftright_2_forward() function:

![image](https://github.com/user-attachments/assets/164e4d7f-80af-4520-ad54-c5e44711c9f2)

Note that the turn_direction_ was calculated once when the go_state(TURN) was called for the robot to turn. For a shorter README, I added the go_state() method directly as a whole. To achieve the desired behavior,
uncomment each section of go_state.

![image](https://github.com/user-attachments/assets/747db8ef-7060-4304-89ea-d76ed2ba62e8)

The get_direction() will specify to turn +- SPEED_ANGULAR according to in which quadrant the obstacle was detected:
![image](https://github.com/user-attachments/assets/7885c607-533a-4a98-af37-249a157035bf)

2.1 For the open loop implementation:
  My robot would turn for a precalculated amount of time based on the angle which has the farthest away osbtacle (by obtaining the index of the max value in ranges array and converting to the corresponding angle)   by using the formula x=vt -> t=x/v. thus, the robot turns until the time has elapsed. 

  The FORWARD state code is the same as in part1.

  The TURN state was updated to:
  ![image](https://github.com/user-attachments/assets/f93e5ad0-6a94-4a8f-a39d-2f59e530f6d5)

  where the turn_direction_ is simply assigned +/- SPEED_ANGULAR at the go_state function (seen in a previous screenshot of the go_state() using the get_direction() function.

  The remaining_time_ is also calculated once the go_state function is called using the calculate_time() function, which would successfully time the robot to keep turning until the angle with the farthest obstacle
  is reached:
  ![image](https://github.com/user-attachments/assets/fb23ec50-b96d-43a8-bbbe-c4735f86147a)

For the closed loop implementation: 
  My robot would turn, without going back, until the range value at the front of the robot indicated there were no obstacles by checking the "is_front_clear()" method.
 
  The forward state was updated to check when to transition from the FORWARD state to the TURN state using check_forward_2_turn():
  
  ![image](https://github.com/user-attachments/assets/21477e04-be84-4973-be05-7f33a175c880)

  The checkforward_2_turn() function checks whether there are obstacles within the range +-pi/4:

  ![image](https://github.com/user-attachments/assets/d9ffd936-349e-45bd-8da2-63b881cc6a34)

  Finally, the TURN state is updated to:

  ![image](https://github.com/user-attachments/assets/3ff098b8-d099-4f21-bcc6-1d4c5a902dd8)


  The turn_direction_ is precalculated in the go_state() function using the calculate_speed(), which simply specifies the direction of the speed:
  
  ![image](https://github.com/user-attachments/assets/7bb6e477-2cd0-4075-a462-dfa4aa2c1849)

  , and the is_front_clear() will keep on checking whether the front is clear or not by checking whether the remaining_time_ has elapse:

  ![image](https://github.com/user-attachments/assets/8bf81af6-1d7f-4d0c-bb5c-b1fe0e94c904)
