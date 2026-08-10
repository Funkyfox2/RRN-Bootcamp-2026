# State-Based Programming
Welcome to the State-Based Programming example!

## Introduction
I'd like to start by mentioning that this example is based off
of work done by Team 2910, Jack in the Bot. With that said, this is an extremely basic rundown. **Things like motor tuning, logging, autonomous programming, and camera systems aren't present here**, as they aren't necessary for this exmaple. Code here is not from an actual robot, so things likely won't work correctly (assuming motors are set up). The example is built off of the CTRE SwerveWithPathplanner Template.

If you would like to dive deeper into state-based programming, I would recommend looking at 2910's codebase for both 2025 and 2026 (though I think 2025 is the better example, as they use more automation).

[2910 - 2025 Codebase](https://github.com/FRCTeam2910/2025CompetitionRobot-Public/)  
[2910 - 2026 Codebase](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/)

## Example Structure
[**RobotContainer**](src/main/java/frc/robot/RobotContainer.java) is in this example to show one way that bindings would be configured.

Each subsystem listed below has adapted this code structure in some way. Typically, you would decide on one of these and then apply it everywhere. Superstructure can be different from the others (having WantedState and CurrentState in Superstructure, but only CurrentState in others).

[**Superstructure**](src/main/java/frc/robot/subsystems/Superstructure.java) - This is the **orchestrator** for the rest of your subsystems. It includes it's own state machines, which then instruct the rest of the subsystems and their state machines.

[**Intake**](src/main/java/frc/robot/subsystems/Intake.java) - A basic over-the-bumper intake that slides. It has one polycarbonate tube roller. This follows similar structure to Superstructure.

[**Shooter**](src/main/java/frc/robot/subsystems/Shooter.java) - Single shooter with static hood. Adjusts shot speed based on distance from target. Opts to remove WantedState control for simplicity, as an example.

[**Spindexer**](src/main/java/frc/robot/subsystems/Spindexer.java) - Indexes game pieces towards the feeder by spinning either itself or a wheel at it's center. Fully commanded by Superstructure, and doesn't have it's own state machines.

[**Feeder**](src/main/java/frc/robot/subsystems/Feeder.java) - Feeds fuel into the shooter. Like the spindexer, it is commanded by Superstructure, and doesn't have it's own state machine.

[**Drivetrain**](src/main/java/frc/robot/subsystems/Drivetrain.java) - Does exist but has not been modified. Again, I would recommend looking at 2910's implementation here.

## Thank you!
Thanks for listening for a few minutes, and I hope that this helps you in some way!  

\- Aidan, Programming Lead of Team 1262, the STAGS

