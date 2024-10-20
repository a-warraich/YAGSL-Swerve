package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

    final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    // Instantiate AbsoluteFieldDrive with joystick inputs
    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> driverXbox.getRightX()  // Assuming right stick X controls the heading
    );

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        if (DriverStation.isTest()) {
            driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            driverXbox.leftBumper().onTrue(Commands.none());
            driverXbox.rightBumper().onTrue(Commands.none());
            drivebase.setDefaultCommand(absoluteFieldDrive); // Set the default command to AbsoluteFieldDrive
        } else {
            driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
            driverXbox.b().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
            driverXbox.start().whileTrue(Commands.none());
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());
            drivebase.setDefaultCommand(absoluteFieldDrive); // Set the default command to AbsoluteFieldDrive
        }
    }

    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setDriveMode() {
        configureBindings();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
