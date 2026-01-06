package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "CrashMain")
public class CrashMain extends CrashOpMode{

    // Setup
    @Override
    protected final void specificSetup() {}

    // MainLoop
    @Override
    protected void mainLoop() {
        crash.drivetrain.moveDrivetrain(-gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x);
        setFlywheelVelocity();
        manualCoreHexAndServoControl();
        intakeArtifact();
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) crash.flywheel).getVelocity());
        telemetry.addData("Flywheel Power", crash.flywheel.getPower());
        telemetry.update();
    }

}
