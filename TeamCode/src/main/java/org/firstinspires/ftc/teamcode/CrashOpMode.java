package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class CrashOpMode extends LinearOpMode {

    //Hardware
    protected CrashHardware crash = new CrashHardware();
    
    ElapsedTime runtime;

    // Main OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware initialization
        crash.init(hardwareMap);

        // Init phase
        generalSetup();
        specificSetup();

        waitForStart();
        runtime.reset();

        // Main loop
        while (opModeIsActive()) {
            mainLoop();
        }

    }

    // Class method
    private void generalSetup() {
        runtime = new ElapsedTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Abstract Methods
    /**
     * To contain anything that must occur specifically in the initialization phase of the specific
     * OpMode
     */
    protected abstract void specificSetup();

    /**
     * To contain anything that must loop and occur after pressing play in the OpMode, can be
     * terminated for an autonomous OpMode using requestOpModeStop()
     */
    protected abstract void mainLoop();

    // Actual active methods
    /**
     * Description: This method contains all the statements that manually control the corehex and
     * the servo
     * Pre-Condition: None
     * Post-Condition: The given command will execute the appropriate corehex or servo motion
     */
    protected void manualCoreHexAndServoControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            crash.coreHex.setPower(0.5);
        } else if (gamepad1.triangle) {
            crash.coreHex.setPower(-0.5);
        }
        // Manual control for the hopper's servo
        if (gamepad1.dpad_left) {
            crash.servo.setPower(1);
        } else if (gamepad1.dpad_right) {
            crash.servo.setPower(-1);
        }
    }

    /**
     * Description: This method contains all the statements that control the flywheel, including
     * manual and automatic launches at set velocities
     * Pre-Condition: None
     * Post-Condition: The given command will execute the appropriate flywheel and related hardware
     * as required
     */
    protected void setFlywheelVelocity() {
        if (gamepad1.start) {
            crash.flywheel.setPower(-0.5);
        }
        else if (gamepad1.left_bumper) {
            farPowerAuto();
        }
        else if (gamepad1.right_bumper) {
            bankShotAuto();
        }
        else if (gamepad1.b) {
            ((DcMotorEx) crash.flywheel).setVelocity(crash.bankVelocity);
        }
        else if (gamepad1.x) {
            ((DcMotorEx) crash.flywheel).setVelocity(crash.maxVelocity);
        }
        else {
            ((DcMotorEx) crash.flywheel).setVelocity(0);
            crash.coreHex.setPower(0);

            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                crash.servo.setPower(0);
            }
        }
    }

    /**
     * Description: Launches an artifact from the close location by running all pieces involved for
     * the duration the button is held
     * Pre-Condition: None
     * Post-Condition: Robot runs all motors and hardware to launch from the close location
     */
    protected void bankShotAuto() {
        ((DcMotorEx) crash.flywheel).setVelocity(crash.bankVelocity);

        if (((DcMotorEx) crash.flywheel).getVelocity() >= crash.bankVelocity - 50) {
            crash.coreHex.setPower(1);
            crash.servo.setPower(-1);
        }
        else {
            crash.coreHex.setPower(-1);
        }
    }

    /**
     * Description: Launches an artifact from the far location by running all pieces involved for
     * the duration the button is held
     * Pre-Condition: None
     * Post-Condition: Robot runs all motors and hardware to launch from the far location
     */
    protected void farPowerAuto() {
        ((DcMotorEx) crash.flywheel).setVelocity(crash.farVelocity);

        if (((DcMotorEx) crash.flywheel).getVelocity() >= crash.farVelocity - 30) {
            crash.coreHex.setPower(1);
            crash.servo.setPower(-1);
        } else {
            crash.coreHex.setPower(-1);
        }
    }

    /**
     * Description: Rotates the intake in or out
     * Pre-Condition: None
     * Post-Condition: Intake is turned or not turned depending on buttons pressed or not pressed
     */
    protected void intakeArtifact() {
        // Intake artifact
        if (gamepad1.right_trigger > 0) {
            crash.intake.setPower(-1.0);
        }
        // Reverse
        else if (gamepad1.left_trigger > 0) {
            crash.intake.setPower(1.0);
        }
        else {
            crash.intake.setPower(0.0);
        }
    }

    /**
     * Description: Turns robot at a specified degree value clockwise from where it is facing
     * Pre-Condition: Param must be a double
     * Post-Condition: Robot turns the specified degrees
     * @param degrees The angle the robot is turning
     */
    protected void turnRobot(double degrees) {

        //Variable
        double targetPosition = (degrees * crash.DRIVE_ENCODER_DEGREE_RATIO) +
                crash.drivetrain.leftFrontDrive.getCurrentPosition();


        while (Math.abs(crash.drivetrain.leftFrontDrive.getCurrentPosition() - targetPosition) > 10) {
            crash.drivetrain.moveDrivetrain(0, 0, 0.5 *
                    (degrees/Math.abs(degrees)));
            telemetry.addData("Target position", targetPosition);
            telemetry.addData("Current position",
                    crash.drivetrain.leftFrontDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Description: Moves the robot a distance facing forward at an angle from where the robot is
     * facing
     * Pre-Condition: Both params are doubles
     * Post-Condition: The robot moves the specified distance at the specified angle
     * @param distanceInches The distance in inches the robot moves
     * @param degrees The degree value of where the robot moves
     */
    protected void moveRobot(double distanceInches, double degrees) {

        degrees = AngleUnit.normalizeDegrees(degrees);
        double theta; // Angle in radians

        if (0 <= degrees && degrees <= 90 || -180 <= degrees && degrees <= -90) {
            theta = Math.toRadians(degrees);

            double targetPosition = ((distanceInches * (Math.cos(theta) * crash.INCHES_TO_ENCODER)
                    + (distanceInches * Math.sin(theta)) * crash.INCHES_TO_ENCODER)) +
                    crash.drivetrain.leftFrontDrive.getCurrentPosition();

            while (Math.abs(crash.drivetrain.leftFrontDrive.getCurrentPosition() - targetPosition) > 10) {
                crash.drivetrain.moveDrivetrain((0.4 * Math.cos(theta)), (0.4 * Math.sin(theta)), 0);
                telemetry.addData("Target position", targetPosition);
                telemetry.addData("Current position", crash.drivetrain.leftFrontDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        else {
            theta = Math.toRadians(degrees);

            double targetPosition = ((distanceInches * (Math.cos(theta) * crash.INCHES_TO_ENCODER)
                    - (distanceInches * Math.sin(theta)) * crash.INCHES_TO_ENCODER)) +
                    crash.drivetrain.rightFrontDrive.getCurrentPosition();

            while (Math.abs(crash.drivetrain.rightFrontDrive.getCurrentPosition() - targetPosition) > 10) {
                crash.drivetrain.moveDrivetrain((0.4 * Math.cos(theta)), (0.4 * Math.sin(theta)), 0);
                telemetry.addData("Target position", targetPosition);
                telemetry.addData("Current position", crash.drivetrain.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }
        }
    }

}
