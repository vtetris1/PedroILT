package org.firstinspires.ftc.teamcode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//ignore this for now
@Autonomous(name="Far_3_Leave_Base")
public class Far_3_Leave_Base extends AutoHardware2 {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    private final double SHOOTER_RPM_SHORT = 2600.0; //3000 // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 3050;
    public Far_3_Leave_Base() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        setAutoDriveMotorModeWithoutEncoder();
        // Set target rpm based on the specific robot position: far, near
        setShooterTargetRpm(SHOOTER_RPM_LONG, SHOOTER_RPM_LONG - 20);
        waitForStart();


        triggerShootStateMachine();

        //reset encoder
        double power = 0.3;
        setDrivePower(-power, -power, -power, -power);
        sleep(2000);
        setDrivePower(0, 0, 0, 0);
    }
}

