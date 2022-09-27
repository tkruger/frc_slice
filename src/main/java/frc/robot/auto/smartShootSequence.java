// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Limelight.LimelightXCommand;
import frc.robot.commands.Indexer.AutoShoot.*;
import frc.robot.commands.Shooter.AutoShoot.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class smartShootSequence extends SequentialCommandGroup {
  /** Creates a new alignedShootSequence. */
  public smartShootSequence(Indexer indexer, Shooter shooter, Drivetrain drivetrain, Limelight limelight, 
      Joystick leftJoystick, Joystick rightJoystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexerDownSlightCommand(indexer), 
      new LimelightXCommand(limelight, drivetrain), 
      new SmartAutoShoot(limelight, shooter),
      new IndexerUpCommand(indexer), 
      new ShooterFlywheelSpinDown(shooter)
    );
  }
}
