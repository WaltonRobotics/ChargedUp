  package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;
import frc.robot.Constants.IntakeK;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX m_intakeMotor = new WPI_TalonFX(0);  // change device number later
  private final Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);  // change channel later

  private final ProfiledPIDController m_intakeController = new ProfiledPIDController(
		IntakeK.kP, 0, IntakeK.kD, IntakeK.kConstraints
	);

  private final GenericEntry nte_intakeMotorFFEffort, nte_intakeMotorPDEffort, 
                             nte_intakeMotorTotalEffort, nte_intakeTargetAngle,
														 nte_intakeActualAngle;


  /** Creates a new Intake. */
  public Intake() {
    DashboardManager.addTab(this);

    m_intakeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    nte_intakeMotorFFEffort = DashboardManager.addTabDial(this, "IntakeMotorFFEffort", -1, 1);
		nte_intakeMotorPDEffort = DashboardManager.addTabDial(this, "IntakeMotorPDEffort", -1, 1);
		nte_intakeMotorTotalEffort = DashboardManager.addTabDial(this, "IntakeMotorTotalEffort", -1, 1);
		nte_intakeTargetAngle = DashboardManager.addTabNumberBar(this, "IntakeTargetAngle",
			IntakeK.kMinAngleDegrees, IntakeK.kMaxAngleDegrees);
		nte_intakeActualAngle = DashboardManager.addTabNumberBar(this, "IntakeActualAngle",
			IntakeK.kMinAngleDegrees, IntakeK.kMaxAngleDegrees);

  }

  public boolean isOpen() {
    return m_intakeSolenoid.get();
  }

  public void setOpen(boolean isOpen) {
    m_intakeSolenoid.set(isOpen);
  }

  public void setDeployed(boolean isDeployed) {
    m_intakeMotor.set(ControlMode.Position, Conversions.degreesToFalcon(
      isDeployed ? 45 : 0, IntakeK.kGearRatio));
  }

  @Override
  public void periodic() {
    //set network table to dashboard here
  }
}
