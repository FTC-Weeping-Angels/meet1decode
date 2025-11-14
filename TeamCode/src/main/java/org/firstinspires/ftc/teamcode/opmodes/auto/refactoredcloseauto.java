package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

//import org.firstinspires.ftc.teamcode.Commands.FollowPath;
//import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Paths.meet1closeauto;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPath;

@Autonomous(name = "red", group = "meh")
public class refactoredcloseauto extends OpModeCommand {
    Robot rob;
    @Override
    public void initialize() {
        rob = new Robot(hardwareMap, Alliance.BLUE);
        meet1closeauto paths = new meet1closeauto(rob);
        rob.follower.setStartingPose(paths.start);

//        rob.intake.Door_init();

        schedule(
               // new RunCommand(rob::periodic),
                new SequentialCommandGroup(
                        new FollowPath(rob, paths.next())
                                .andThen(
//                                        new Shoot(rob),
//                                        new Shoot(rob),
//                                        new Shoot(rob)
                                )
                )
        );
  }

    @Override
    public void start() {
       // rob.intake.idle();
    }

    @Override
    public void stop(){rob.stop();}
}
