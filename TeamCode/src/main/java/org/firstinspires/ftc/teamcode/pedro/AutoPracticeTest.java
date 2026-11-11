package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import android.view.KeyEvent;

@Autonomous
public class AutoPracticeTest extends OpMode {
    private Follower follower;
    PathState pathState;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0); // These are both subject to change
    private final Pose park = p.of(48, 48, 90); // This park position is important. cus must be here to shoot
    // other poses...
    // private final Pose controlPose = p.of(36, 60, 45); // This stuff is just for curved paths

    @Override
    public void init(){
        Scheduler.reset();
        //  follower = Constants.create(hardwareMap); // creates a follower using the drivetrain, localizer, and algorithm
        follower.setPose(startPose);
        follower.update();
    }
    private Path startToScore() {
        return line(startPose, park).linear(startPose, park);
    }
    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    public void start() {
        schedule(follow(follower, park()));
        schedule(autoRoutine());
        // using the same park path from the last page
    }

    public enum PathState{  // This will keep track of what state we are in and helps with debugging basically doing this for organization.
        //start end states
        //drive then shoot state
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }



    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

}
