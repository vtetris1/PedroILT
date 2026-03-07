package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far 3rd row + corner balls", group = "Examples")
public class AutoPedroBlueFar3rdRow extends AutoPedroCommon {


    /*
    private final Pose blue_1st_row_start = new Pose(50, 78, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose blue_1st_row_end = new Pose(25, 78, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose blue_2nd_row_start = new Pose(50, 54, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose blue_2nd_row_end = new Pose(25, 54, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose blue_gate = new Pose(20, 60, Math.toRadians(180));
    private final Pose blue_3rd_row_start = new Pose(50, 30, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose blue_3rd_row_end = new Pose(25, 30, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose blue_parking = new Pose(41, 38, Math.toRadians(0));

     */
    public void setPath(){
        poseList.add(new PoseItem(blue_near_init,"none","shoot_preload"));
        poseList.add(new PoseItem(blue_near_score,"shoot","intake_3rd_row"));
        poseList.add(new PoseItem(blue_3rd_row_start,"intake","intake_3rd_row"));
        poseList.add(new PoseItem(blue_3rd_row_end,"none","intake_3rd_row"));
        poseList.add(new PoseItem(blue_near_score,"shoot","turn_left_1"));
        poseList.add(new PoseItem(blue_near_turn_left,"intake","intake_1"));
        poseList.add(new PoseItem(blue_near_corner,"none","shoot_1"));
        poseList.add(new PoseItem(blue_near_score,"shoot","shoot_1"));
        poseList.add(new PoseItem(blue_near_turn_left,"intake","intake_1"));
        poseList.add(new PoseItem(blue_near_corner,"none","shoot_1"));
        poseList.add(new PoseItem(blue_near_score,"shoot","intake_2"));

        /*
        pathList.add(new PoseItem(blue_far_score,blue_2nd_row_start,"none","go_to_2nd_row"));
        pathList.add(new PoseItem(blue_2nd_row_start,blue_2nd_row_end,"intake","intake_2nd_row"));
        pathList.add(new PoseItem(blue_2nd_row_end,blue_gate,"none","shoot_2nd_row"));
        pathList.add(new PoseItem(blue_gate,blue_far_score,"shoot","shoot_2nd_row"));

        pathList.add(new PoseItem(blue_far_score,blue_1st_row_start,"none","go_to_1st_row"));
        pathList.add(new PoseItem(blue_1st_row_start,blue_1st_row_end,"intake","intake_1st_row"));
        pathList.add(new PoseItem(blue_1st_row_end,blue_far_score,"shoot","shoot_1st_row"));

        pathList.add(new PoseItem(blue_far_score,blue_3rd_row_start,"none","go_to_3rd_row"));
        pathList.add(new PoseItem(blue_3rd_row_start,blue_3rd_row_end,"intake","intake_3rd_row"));
        pathList.add(new PoseItem(blue_3rd_row_end,blue_far_score,"shoot","shoot_3rd_row"));
        pathList.add(new PoseItem(blue_far_score,blue_parking,"none","parking"));

         */
    }
}
