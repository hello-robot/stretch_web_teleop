import { RemoteRobot } from "shared/remoterobot";
import { VelocityCommand } from "shared/commands";
import { FEEDING_CONFIGURATION, STOW_WRIST_GRIPPER, ValidJoints } from "shared/util";
import { ActionMode, TaskMode } from "../utils/component_definitions";

/**
 * Provides logic to connect the {@link RemoteRobot} and the components in the
 * interface
 */
export abstract class FunctionProvider {
    protected static remoteRobot?: RemoteRobot;
    public static velocityScale: number;
    public static actionMode: ActionMode;
    public static taskMode: TaskMode;
    public activeVelocityAction?: VelocityCommand;
    public velocityExecutionHeartbeat?: number; // ReturnType<typeof setInterval>

    /**
     * Adds a remote robot instance to this function provider. This must be called
     * before any components of the interface will be able to execute functions
     * to change the state of the robot.
     *
     * @param remoteRobot the remote robot instance to add
     */
    static addRemoteRobot(remoteRobot: RemoteRobot) {
        FunctionProvider.remoteRobot = remoteRobot;
    }

    /**
     * Sets the initial values for the velocity scale and action mode
     *
     * @param velocityScale initial velocity scale
     * @param actionMode initial action mode
     */
    static initialize(velocityScale: number, actionMode: ActionMode) {
        this.velocityScale = velocityScale;
        this.actionMode = actionMode;
        this.taskMode = TaskMode.Default;
    }

    public incrementalBaseDrive(linVel: number, angVel: number) {
        this.stopCurrentAction();
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
            linVel,
            angVel,
        );
    }

    public incrementalJointMovement(jointName: ValidJoints, increment: number) {
        this.stopCurrentAction();
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment);
    }

    public continuousBaseDrive(linVel: number, angVel: number) {
        this.stopCurrentAction();
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
            linVel,
            angVel,
        );
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
                linVel,
                angVel,
            );
        }, 150);
    }

    public continuousJointMovement(jointName: ValidJoints, increment: number) {
        this.stopCurrentAction();
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment);
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction =
                FunctionProvider.remoteRobot?.incrementalMove(
                    jointName,
                    increment,
                );
        }, 150);
    }

    // NOTE: When we undo this temp fix (of not stopping the
    // trajectory client) we also need to undo it in robot.jsx
    // `stopExecution()`.
    public stopCurrentAction(send_stop_command: boolean = false) {
        if (send_stop_command) FunctionProvider.remoteRobot?.stopTrajectory();
        if (this.activeVelocityAction) {
            // TODO: this.activeVelocityAction.stop sometimes (always?) executes the
            // exact same cancellation command(s) as FunctionProvider.remoteRobot?.stopTrajectory,
            // which means we are unnecessarily calling it twice.
            if (send_stop_command) this.activeVelocityAction.stop();
            this.activeVelocityAction = undefined;
        }
        if (this.velocityExecutionHeartbeat) {
            clearInterval(this.velocityExecutionHeartbeat);
            this.velocityExecutionHeartbeat = undefined;
        }
    }

    static updateTaskMode() {
        this.taskMode = this.taskMode == TaskMode.Feeding ? TaskMode.Default : TaskMode.Feeding
        if (this.taskMode == TaskMode.Feeding) {
            FunctionProvider.remoteRobot?.setRobotPose(
                FEEDING_CONFIGURATION,
            )
            FunctionProvider.remoteRobot.setToggle("setFeedingMode", true)
        } else {
            FunctionProvider.remoteRobot.setToggle("setFeedingMode", false)
        }
    }
}
