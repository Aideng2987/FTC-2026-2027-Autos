package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Auto 1", group = "Pedro")
public class Auto_1 extends OpMode{
    private Follower follower;

    private final Container robot = new Container();

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose start = p.of(58.75, 13.75, 90);
    private final Pose shoot = p.of(58.75, 12.75, 90);
    private final Pose intake = p.of(12.75, 10.75, 180);
    private final Pose turn = p.of(17.75, 21.75, 180);
    private final Pose shootAgain = p.of(58.75, 13.75, 90);
    private final Pose bumpIntoFlower = p.of(14.75, 43.75, -34.2869);
    private final Pose parking = p.of(10.75, 100.75, -85.9858);


    public Path shoot() {
        return Paths.line(start, shoot).linear(start, shoot);
    }


    public Path intake() {
        return Paths.line(shoot, intake).constant(intake);
    }


    public Path turn() {
        return Paths.line(intake, turn).constant(turn);
    }


    public Path shootAgain() {
        return Paths.line(turn, shootAgain).constant(shootAgain);
    }


    public Path bumpIntoFlower() {
        return Paths.line(shootAgain, bumpIntoFlower).reverseTangent();
    }
    public Path parking() {
        return Paths.line(bumpIntoFlower, parking).reverseTangent();
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void start() {
        schedule(follow(follower,shoot()));
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    private Command autoRoutine() {
        return sequential( // sequential is basically a sequence that the robot has to do
                follow(follower, shoot()),
                robot.shootForMs(1500),
                parallel(  // parrallel is basically multiple things the robot has to do at the same time
                        sequential(
                                follow(follower, intake()),
                                follow(follower, turn())
                        ),
                        robot.intakeForMs(3000, 1.0)
                ),
                follow(follower, shootAgain()),
                robot.shootForMs(1500),
                follow(follower, bumpIntoFlower()),
                follow(follower, parking())
        );
    }
}
