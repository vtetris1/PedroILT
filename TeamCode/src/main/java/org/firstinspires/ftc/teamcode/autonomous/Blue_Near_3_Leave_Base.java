package org.firstinspires.ftc.teamcode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

//ignore this for now
@Autonomous(name="BLUE_Near_3_Leave_Base")
public class Blue_Near_3_Leave_Base extends AutoHardware2 {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    private final double SHOOTER_RPM_SHORT = 3000.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 3880;
    private int target = 200; // Rotations Per Minute

    public Blue_Near_3_Leave_Base() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        //reset encoder
        setAutoDriveMotorMode();
        // Set target rpm based on the specific robot position: far, near
        setShooterTargetRpm(2900);
        waitForStart();

        double inchesForward = 50;
        int ticksForward = (int)(inchesForward * getTicksPerInch());
        driveMotors(ticksForward, ticksForward, ticksForward, ticksForward, 0.3, true, getCurrentYaw());

        triggerShoot(3);

        inchesForward = 20;
        ticksForward = (int)(inchesForward * getTicksPerInch());
        driveStrafe(ticksForward,ticksForward,ticksForward,ticksForward,0.6, true, getCurrentYaw());

    }
}

