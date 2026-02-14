package org.firstinspires.ftc.teamcode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//ignore this for now
@Autonomous(name="RED_Near_3_Leave_Base")
public class RED_Near_3_Leave_Base extends AutoHardware2 {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    private final double SHOOTER_RPM_SHORT = 3150.0;
    private int target = 200; // Rotations Per Minute

    public RED_Near_3_Leave_Base() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        //reset encoder
        setAutoDriveMotorMode();
        // Set target rpm based on the specific robot position: far, near
        setShooterTargetRpm(SHOOTER_RPM_SHORT, SHOOTER_RPM_SHORT - 20);
        waitForStart();

        double inchesForward = 40;
        int ticksForward = -(int) (inchesForward * getTicksPerInch());
        driveMotors(ticksForward, ticksForward, ticksForward, ticksForward, 0.3, false, getCurrentYaw());

        triggerShoot(3);

        double targetYaw = -45;
        turnToTargetYaw(targetYaw, 0.5, 3000);
        inchesForward = 26;
        ticksForward = (int) (inchesForward * getTicksPerInch());
        driveMotors(ticksForward, ticksForward, ticksForward, ticksForward, 0.3, false, getCurrentYaw());
    }
}

