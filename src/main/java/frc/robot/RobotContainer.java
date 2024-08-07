package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.FlashingLEDColor;
import frc.robot.commands.MovingColorLEDs;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.lightsabers.Lightsaber;
import frc.robot.subsystems.lightsabers.LightsaberIO;
import frc.robot.subsystems.lightsabers.LightsaberReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretReal;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.driverID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve s_Swerve;
    private Lightsaber s_Lightsabers;
    private Turret s_Turret;
    private LEDs leds1 = new LEDs(0, 60);
    private LEDs leds2 = new LEDs(60, 120);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotRunType runtimeType) {
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        switch (runtimeType) {
            case kReal:
                s_Swerve = new Swerve(new SwerveReal());
                s_Lightsabers = new Lightsaber(new LightsaberReal());
                s_Turret = new Turret(new TurretReal());
                break;
            case kSimulation:
                // drivetrain = new Drivetrain(new DrivetrainSim() {});
                s_Swerve = new Swerve(new SwerveIO() {});
                s_Lightsabers = new Lightsaber(new LightsaberIO() {});
                s_Turret = new Turret(new TurretIO() {});
                break;
            default:
                s_Swerve = new Swerve(new SwerveIO() {});
                s_Lightsabers = new Lightsaber(new LightsaberIO() {});
                s_Turret = new Turret(new TurretIO() {});
        }
        // Configure the button bindings
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
        leds1.setDefaultCommand(leds1.setStaticColor(Color.kBlack));
        leds2.setDefaultCommand(leds2.setStaticColor(Color.kBlack));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driver.a().toggleOnTrue(s_Lightsabers.turnLightsabers(.4));
        driver.b().toggleOnTrue(
            s_Turret.turnBackandForth(.1).alongWith(s_Lightsabers.turnLightsabers(.4)));
        driver.x().toggleOnTrue(new FlashingLEDColor(leds1, Color.kRed, Color.kBlack, 60)
            .alongWith(new MovingColorLEDs(leds2, Color.kWhite, 3, false)));
        driver.y().onTrue(Commands.runOnce(() -> s_Swerve.resetModulesToAbsolute()));
        driver.povRight().whileTrue(s_Turret.turnTurretClockwise(.1));
        driver.povLeft().whileTrue(s_Turret.turnTurretCounterClockwise(.1));
        driver.rightTrigger().whileTrue(new TeleopSwerve(s_Swerve, driver, false, false, 4));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "wait":
                autocommand = new WaitCommand(1.0);
                break;
            default:
                autocommand = new InstantCommand();
        }
        return autocommand;
    }
}
