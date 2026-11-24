package org.firstinspires.ftc.teamcode.pedro;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

@Autonomous(name = "Auto 1", group = "Pedro")
public class Auto_1 extends OpMode{
    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    // These 2 are subject to change
    //(x,y,z)  x= x cords y = y coords z == degrees for where the robot is facing
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
    }
    //poses from before

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }
}
