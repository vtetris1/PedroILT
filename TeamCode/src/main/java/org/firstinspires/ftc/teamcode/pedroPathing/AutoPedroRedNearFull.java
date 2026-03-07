package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Near 1,2,3 rows ", group = "Examples")
public class AutoPedroRedNearFull extends AutoPedroCommon {
 //
    public void setPath(){
        poseList.add(new PoseItem(red_far_init,"none","shoot_preload"));
        poseList.add(new PoseItem(red_far_score,"shoot","go_to_1st_row"));
        poseList.add(new PoseItem(red_1st_row_start,"intake","intake_1st_row"));
        poseList.add(new PoseItem(red_1st_row_end,"none","shoot_1st_row"));
        poseList.add(new PoseItem(red_far_score,"shoot","go_to_2nd_row"));
        poseList.add(new PoseItem(red_2nd_row_start,"intake","intake_2nd_row"));
        poseList.add(new PoseItem(red_2nd_row_end,"none","shoot_2nd_row"));
        poseList.add(new PoseItem(red_far_score,"shoot","go_to_3rd_row"));
        poseList.add(new PoseItem(red_3rd_row_start,"intake","intake_3rd_row"));
        poseList.add(new PoseItem(red_3rd_row_end,"none","shoot_3rd_row"));
        poseList.add(new PoseItem(red_far_score,"shoot","parking"));
    }
}
