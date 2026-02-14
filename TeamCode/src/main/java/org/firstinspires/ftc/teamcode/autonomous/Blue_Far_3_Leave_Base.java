package org.firstinspires.ftc.teamcode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.AutoHardware2;

//ignore this for now
@Autonomous(name="Far_3_Leave_Base")
public class Blue_Far_3_Leave_Base extends AutoHardware2 {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    double shooter_rpm_far = 3000;  // Rotations Per Minute

    public Blue_Far_3_Leave_Base() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        //reset encoder
        setAutoDriveMotorMode();
        // Set target rpm based on the specific robot position: far, near
        setShooterTargetRpm(shooter_rpm_far);
        waitForStart();
        triggerShoot(3);

        double targetYaw = getCurrentYaw();
        double inchesForward = 16;
        int ticksForward = (int)(inchesForward * getTicksPerInch());
        setDrivePower(0.3, 0.3, 0.3, 0.3);
        sleep(1500);
        setDrivePower(0, 0, 0, 0);
        sleep(30000);
    }
}

