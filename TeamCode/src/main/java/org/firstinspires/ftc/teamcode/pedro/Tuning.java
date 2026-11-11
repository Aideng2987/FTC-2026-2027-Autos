package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import com.pedropathing.tuning.autotune.Procedure;
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
