// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //define Joysticks
  private final Joystick r_joystick = new Joystick(0);
  //Define buttons
  //pos1 button
  private final JoystickButton pos_1_button = new JoystickButton(r_joystick, 0);
  //pos2 button
  private final JoystickButton pos_2_button = new JoystickButton(r_joystick, 1);

  //define motor stuff
  //define motor
  CANSparkMax pivot_motor = new CANSparkMax(Constants.pivot_motor_id, MotorType.kBrushless);
  //creates voltage for pivot motor
  //volt for pos1
  double pivot_volt_1;
  //volt for pos3
  double pivot_volt_2;

  //pivot potentiometer stuff
  //define potentiometer
  AnalogPotentiometer pivot_pot = new AnalogPotentiometer(0, 180, 0);
  //pivot potentiometer current value variable
  double pivot_pos;

  //define pid loop
  PIDController pivot_pid_cont = new PIDController(Constants.pivot_kp, Constants.pivot_ki, Constants.pivot_kd);

  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pivot_motor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pivot Volt", pivot_motor.get());
    SmartDashboard.putNumber("Pivot Position", pivot_pot.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //gets current position of pivot using the potentiometer
    pivot_pos = pivot_pot.get();

    //creates voltage for motor using pid controller
    //pos1
    pivot_volt_1 = pivot_pid_cont.calculate(pivot_pos, Constants.position_1);
    //pos2
    pivot_volt_2 = pivot_pid_cont.calculate(pivot_pos, Constants.position_2);

    //sets motor voltage
    if (pos_1_button.getAsBoolean()) {
      pivot_motor.setVoltage(pivot_volt_1);
    } else if (pos_2_button.getAsBoolean()) {
      pivot_motor.setVoltage(pivot_volt_2);
    } else {
      pivot_motor.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
