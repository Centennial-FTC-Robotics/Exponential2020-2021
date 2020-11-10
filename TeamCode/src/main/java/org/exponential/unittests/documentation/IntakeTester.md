# DrivetrainTester
### Test case 0: Autonomous Intaking
Runs the intake() method, meant for autonomous usage, on the intake for 3 seconds.

### Test case 1: Autonomous Outtaking
Runs the outtake() method, meant for autonomous usage, on the intake for 3 seconds.

### Test case 2: Teleop Intaking
Runs intake.setPowerInput(-1), simulating "down" on gamepad1.y which should intake for 3 seconds.

### Test case 3: Teleop Outtaking
Runs intake.setPowerInput(1), simulating "up" on gamepad1.y which should outtake for 3 seconds.


