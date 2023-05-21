import { RemoteRobot } from "shared/remoterobot"
import { VelocityCommand } from 'shared/commands'
import { ActionMode } from "../staticcomponents/actionmodebutton"
import { ButtonPadFunction, ButtonFunctions } from "../layoutcomponents/buttonpads"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints } from 'shared/util'
import { PredictiveDisplayFunctions } from '../layoutcomponents/predictivedisplay'
import { DEFAULT_VELOCITY_SCALE } from "../staticcomponents/velocitycontrol"

/**
 * Provides logic to connect the {@link RemoteRobot} and the components in the 
 * interface
 */
export abstract class FunctionProvider {
    protected static remoteRobot?: RemoteRobot;
    public static velocityScale: number;
    public static actionMode: ActionMode;

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
     * Sets the initial values for velocity scale and action mode.
     */
    static initialize() {
        this.velocityScale = DEFAULT_VELOCITY_SCALE;
        this.actionMode = ActionMode.StepActions;
    }
}

/**
 * Provides functions for the button pads
 */
export class ButtonFunctionProvider extends FunctionProvider {
    private activeVelocityAction?: VelocityCommand;
    private velocityExecutionHeartbeat?: number // ReturnType<typeof setInterval>

    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    private incrementalBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
    }

    private incrementalArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction = FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
    }

    private continuousBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.driveBase(linVel, angVel),
            this.velocityExecutionHeartbeat = window.setInterval(() => {
                this.activeVelocityAction =
                    FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
            }, 150);
    }

    private continuousArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction =
                FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        }, 150);
    }

    private stopCurrentAction() {
        if (this.activeVelocityAction) {
            // No matter what region this is, stop the currently running action
            this.activeVelocityAction.stop()
            this.activeVelocityAction = undefined
            clearInterval(this.velocityExecutionHeartbeat)
            this.velocityExecutionHeartbeat = undefined
            // clearTimeout(this.activeVelocityActionTimeout)
            // this.activeVelocityActionTimeout = undefined
        }
    }

    /**
     * Takes a ButtonPadFunction which indicates the type of button (e.g. drive 
     * base forward, lift arm), and returns a set of functions to execute when 
     * the user interacts with the button.
     * 
     * @param buttonPadFunction the {@link ButtonPadFunction}
     * @returns the {@link ButtonFunctions} for the button
     */
    public provideFunctions(buttonPadFunction: ButtonPadFunction): ButtonFunctions | PredictiveDisplayFunctions {
        switch (FunctionProvider.actionMode) {
            case ActionMode.StepActions:
                switch (buttonPadFunction) {
                    case ButtonPadFunction.BaseForward:
                        return {
                            onClick: () => this.incrementalBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case ButtonPadFunction.BaseReverse:
                        return {
                            onClick: () => this.incrementalBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateLeft:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.PredictiveDisplay:
                        return {
                            onClick: (length: number, angle: number) => console.log('predictive display click, length:', length, "angle:", angle),
                            onMove: (length: number, angle: number) => console.log('predictive display move, length:', length, "angle:", angle),
                            onRelease: () => console.log("predictive display on release")
                        } as PredictiveDisplayFunctions;
                }
                break;
            case ActionMode.PressRelease:
                switch (buttonPadFunction) {
                    case ButtonPadFunction.BaseForward:
                        return {
                            onClick: () => this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseReverse:
                        return {
                            onClick: () => this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateLeft:
                        return {
                            onClick: () => this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight:
                        return {
                            onClick: () => this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower:
                        return {
                            onClick: () => this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift:
                        return {
                            onClick: () => this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend:
                        return {
                            onClick: () => this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract:
                        return {
                            onClick: () => this.continuousArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn:
                        return {
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut:
                        return {
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen:
                        return {
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose:
                        return {
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onRelease: () => this.stopCurrentAction(),
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
            case ActionMode.ClickClick:
                switch (buttonPadFunction) {
                    case ButtonPadFunction.BaseForward:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseReverse:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateLeft:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose:
                        return {
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
        }

        return {
            onClick: () => console.log(`${FunctionProvider.actionMode} ${buttonPadFunction} clicked`),
            onRelease: () => console.log(`${FunctionProvider.actionMode} ${buttonPadFunction} releasd`)
        }
    }
}