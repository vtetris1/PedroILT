package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class PoseItem {
    Pose pose;
    String action;
    String state;

    public PoseItem(Pose pose, String action, String state) {
        this.pose = pose;
        this.action = action;
        this.state = state;
    }
}
