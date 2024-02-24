// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Crashboard;

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax motor;

  private double targetPosition = 0;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    
    motor = new CANSparkMax(2, MotorType.kBrushless);

    if (!RobotBase.isReal())
    {
      REVPhysicsSim.getInstance().addSparkMax(motor,DCMotor.getNEO(1));

      //SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1),100.0, 2.0, 0.0, 180.0, true);
    }
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void SetVoltage(double speed)
  {
    //motor.set(.3);
    motor.setVoltage(speed);
  }

  public void MoveToPosition(double position)
  {

  }

  public double GetPosition()
  {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //REVPhysicsSim.getInstance().run();
    var output = motor.getAppliedOutput();
    Crashboard.toDashboard("Applied Output", output, "Default");
  }
}
