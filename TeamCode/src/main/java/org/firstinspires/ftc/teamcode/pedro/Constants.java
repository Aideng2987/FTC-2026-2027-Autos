package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.revhub.localizers.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.revhub.drivetrains.MecanumConfig;



public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("topLeftWheel");
                c.backLeftName.set("bottomLeftWheel");
                c.frontRightName.set("topRightWheel");
                c.backRightName.set("bottomRightWheel");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            }
    );


}