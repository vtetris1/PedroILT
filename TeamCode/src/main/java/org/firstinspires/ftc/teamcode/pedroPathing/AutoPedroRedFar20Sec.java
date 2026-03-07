package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far Wait 20 seconds to shoot", group = "Examples")
public class AutoPedroRedFar20Sec extends AutoPedroCommon {

    /*
    private final Pose red_1st_row_start = new Pose(50, 78, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose red_1st_row_end = new Pose(25, 78, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose red_2nd_row_start = new Pose(50, 54, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose red_2nd_row_end = new Pose(25, 54, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose red_gate = new Pose(20, 60, Math.toRadians(180));
    private final Pose red_3rd_row_start = new Pose(50, 30, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose red_3rd_row_end = new Pose(25, 30, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose red_parking = new Pose(41, 38, Math.toRadians(0));

     */
    public void setPath(){
        poseList.add(new PoseItem(red_near_init,"none","shoot_preload"));
        poseList.add(new PoseItem(red_near_score,"sleep_and_shoot","turn_left_1"));
        poseList.add(new PoseItem(red_near_turn_left,"intake","intake_1"));
        poseList.add(new PoseItem(red_near_corner,"none","shoot_1"));
        poseList.add(new PoseItem(red_near_score,"shoot","shoot_1"));
        //pathList.add(new PathItem(red_near_score,red_near_turn_left,"none","shoot_1"));
        //pathList.add(new PathItem(red_near_score,red_near_corner,"intake","intake_2"));
        //pathList.add(new PathItem(red_near_corner,red_near_score,"shoot","shoot_2"));
        /*
        pathList.add(new PathItem(red_far_score,red_2nd_row_start,"none","go_to_2nd_row"));
        pathList.add(new PathItem(red_2nd_row_start,red_2nd_row_end,"intake","intake_2nd_row"));
        pathList.add(new PathItem(red_2nd_row_end,red_gate,"none","shoot_2nd_row"));
        pathList.add(new PathItem(red_gate,red_far_score,"shoot","shoot_2nd_row"));

        pathList.add(new PathItem(red_far_score,red_1st_row_start,"none","go_to_1st_row"));
        pathList.add(new PathItem(red_1st_row_start,red_1st_row_end,"intake","intake_1st_row"));
        pathList.add(new PathItem(red_1st_row_end,red_far_score,"shoot","shoot_1st_row"));

        pathList.add(new PathItem(red_far_score,red_3rd_row_start,"none","go_to_3rd_row"));
        pathList.add(new PathItem(red_3rd_row_start,red_3rd_row_end,"intake","intake_3rd_row"));
        pathList.add(new PathItem(red_3rd_row_end,red_far_score,"shoot","shoot_3rd_row"));
        pathList.add(new PathItem(red_far_score,red_parking,"none","parking"));

         */
    }
}
