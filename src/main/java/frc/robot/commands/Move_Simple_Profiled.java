// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This code is based on: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html#complete-usage-example

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorSub;

public class Move_Simple_Profiled extends CommandBase {
  private final MotorSub m_motorSub;
  private double m_PositionGoal;

  // Creates a new set of trapezoidal motion profile constraints
  public double MaxMotorVelocity = 100.0; // Max velocity in RPM
  public double MaxMotorAccel = 30.0; // Max acceleration in revolutions per second squared
  private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(MaxMotorVelocity, MaxMotorAccel);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_start = new TrapezoidProfile.State();

  private TrapezoidProfile m_profile;

  private static double TimeStep = 0.02; // 20ms per scheduler run
  private double ClockTick = 0;

  /** Creates a new Move_Simple_Profiled. */
  public Move_Simple_Profiled(double PositionGoal, MotorSub subsystem) {
    m_PositionGoal = PositionGoal;
    // Use addRequirements() here to declare subsystem dependencies.
    m_motorSub = subsystem;
    addRequirements(m_motorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // START DEBUG: Get limits from Shuffleboard
    MaxMotorVelocity = SmartDashboard.getNumber("MaxMotorVelocity", 100.0);
    MaxMotorAccel = SmartDashboard.getNumber("MaxMotorAccel", 30.0);
    // Required to get text boxes in the window if not there already
    SmartDashboard.putNumber("MaxMotorVelocity", MaxMotorVelocity);
    SmartDashboard.putNumber("MaxMotorAccel", MaxMotorAccel);
    m_constraints = new TrapezoidProfile.Constraints(MaxMotorVelocity, MaxMotorAccel);
    // END DEBUG: Get limits from Shuffleboard

    // Profile states contain both position and velocity!
    m_goal = new TrapezoidProfile.State(m_PositionGoal, 0);
    m_start = new TrapezoidProfile.State(m_motorSub.getPosition(), 0);

    // Create a motion profile limited by the constraints.
    m_profile = new TrapezoidProfile(m_constraints, m_goal, m_start);

    ClockTick = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_motorSub.setPositionGoal(m_profile.calculate(ClockTick).position);
    ClockTick = ClockTick + TimeStep;  // Advance time for next calculation
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motorSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_profile.calculate(ClockTick) == m_goal) {
      return true;
    } else return false;
  }
}
