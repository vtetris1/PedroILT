package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class PathItem {
    Pose pose1;
    Pose pose2;
    String action;
    String state;

    public PathItem(Pose pose1, Pose pose2, String action, String state) {
        this.pose1 = pose1;
        this.pose2 = pose2;
        this.action = action;
        this.state = state;
    }
}
