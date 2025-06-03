import { RemoteRobot } from "shared/remoterobot";
import { VelocityCommand } from "shared/commands";
import { ValidJoints } from "shared/util";
import { ActionModeType } from "../utils/component_definitions";
import { ButtonPadButton } from "./ButtonFunctionProvider";

/**
 * Provides logic to connect the {@link RemoteRobot} and the components in the
 * interface
 */
export abstract class FunctionProvider {
    protected static remoteRobot?: RemoteRobot;
    public static velocityScale: number;
    public static actionMode: ActionModeType;
    public activeButtonPadFunction: ButtonPadButton;
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
    static initialize(velocityScale: number, actionMode: ActionModeType) {
        this.velocityScale = velocityScale;
        this.actionMode = actionMode;
    }

    public incrementalBaseDrive(linVelX: number, linVelY: number, angVel: number) {
        this.stopCurrentAction();
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
            linVelX,
            linVelY,
            angVel,
        );
    }

    public incrementalJointMovement(jointName: ValidJoints, increment: number) {
        this.stopCurrentAction();
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment);
    }

    public continuousBaseDrive(linVelX: number, linVelY: number, angVel: number) {
        this.stopCurrentAction();
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
            linVelX,
            linVelY,
            angVel,
        );
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(
                linVelX,
                linVelY,
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
        // if (send_stop_command) FunctionProvider.remoteRobot?.stopTrajectory();
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
}
