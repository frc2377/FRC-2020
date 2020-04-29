# C Company (FRC Team 2377) 2020 Robot Code

This is the repository of C Company's 2020 Robot named *Old-Bay-wan-Kenobi*.

## Controls and Button Mappings

### Default Commands

- Drive is set to ArcadeMode
  - Left stick controls speed
  - Right stick controls direction
- Conveyor will collect Power Cells until it contains 5
  - When manipulator fires the cannon, the conveyor will run until it is empty and then return to default behavior.
  - If the driver picks up a Power Cell while the manipulator is firing, it will continue to run until the manipulator stops firing.

### Driver

- **Drive** is set up as Arcade on an Xbox Controller
- **Left Bumper**: Retract Intake
- **Right Bumper**: Deploy Intake

### Manipulator

- **Left Trigger**: Hold left trigger while moving right stick on the X axis to manually move the turret.  When released, the turret will continue to auto align.
- **Right Trigger**: Fire cannon (if Limelight sees a target)
- **Y**: Toggle Control Panel Arm
- **Left Bumper**: Activate Stage 2 Control Panel Command
- **Right Bumper**: Activate Stage 3 Control Panel Command

### Todo

- Add Climbing code
- Incorporate LEDs