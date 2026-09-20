// i dont know what this was meant for it was never used in the original code



// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.Constants.IntakeConstants;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ToggleIntake extends Command {
//   public IntakeSubsystem intake;

//   /** Creates a new ToggleIntake. */
//   public ToggleIntake( IntakeSubsystem _intake ) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.intake = _intake;
//     this.addRequirements(this.intake);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (this.intake.currentState == IntakeConstants.IntakeState.DEPLOYED) {
//       this.intake.currentState = IntakeConstants.IntakeState.STOWED;
//     } else {
//       this.intake.currentState = IntakeConstants.IntakeState.DEPLOYED;
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (this.intake.currentState == IntakeConstants.IntakeState.DEPLOYED) {
//       this.intake.deployLeaderMotor.set(-IntakeConstants.deploySpeed);
//     } else {
//       this.intake.deployLeaderMotor.set(IntakeConstants.deploySpeed);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     this.intake.deployLeaderMotor.set(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if (this.intake.currentState == IntakeConstants.IntakeState.DEPLOYED) {
//       return this.intake.deployEncoder.getPosition() < IntakeConstants.deployPosition;
//     } else {
//       return this.intake.deployEncoder.getPosition() > IntakeConstants.stowedPosition;
//     }
//   }
// }