# DrivetrainTester
### Test case 0: Regular Strafing
Moves the robot for a foot at 0° and then back to the original position. Then at 45°, 90°, etc. until 315°.
Stays facing "forward" (which is the 90° direction)

![Regular Strafing Diagram](images/drivetrain_regular_strafing_diagram.png)

### Test case 1: Field Centric 45° Strafing
Does the same strafing pattern as test case 0, but while the robot is facing the 45° direction (top right)

### Test case 2: Field Centric 202.5° Strafing
Does the same strafing pattern as test case 0, but while the robot is facing the 202.5° direction (left bottom left)

### Test case 3: Odometric Position Tracking
Moves diagonally to the top left (1 foot horizontally, 1 foot vertically), left 2 feet, down 2 feet, right 2 feet, up 2 feet, and then diagonally back to the original position

![Odometric Position DIagram](images/odometric_position_diagram.png)
