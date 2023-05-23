import { FunctionProvider } from "./functionprovider"
import { VoiceCommandFunction, VoiceCommandFunctions } from "../staticcomponents/voicecommands"
import { ActionMode } from "../staticcomponents/actionmodebutton"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints } from 'shared/util'

export class VoiceFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    /**
    * Takes a VoiceCommandFunction which indicates the type of action (e.g. drive 
    * base forward, lift arm), and returns a set of functions to execute when 
    * the user says a command.
    * 
    * @param voiceCommandFunction the {@link VoiceCommandFunction}
    * @returns the {@link VoiceCommandFunctions} for the spoken command
    */
    public provideFunctions(voiceCommandFunction: VoiceCommandFunction): VoiceCommandFunctions {
        switch (voiceCommandFunction) {
            case VoiceCommandFunction.BaseForward:
                return {
                    command: "drive forward",
                    callback: () => this.incrementalBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0)
                }
            case VoiceCommandFunction.BaseReverse:
                return {
                    command: "drive backward",
                    callback: () => this.incrementalBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0),
                }
            case VoiceCommandFunction.BaseRotateLeft:
                return {
                    command: "rotate robot left",
                    callback: () => this.incrementalBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.BaseRotateRight:
                return {
                    command: "rotate robot right",
                    callback: () => this.incrementalBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.ArmLower:
                return {
                    command: "lower arm",
                    callback: () => this.incrementalArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.ArmLift:
                return {
                    command: "raise arm",
                    callback: () => this.incrementalArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.ArmExtend:
                return {
                    command: "extend arm",
                    callback: () => this.incrementalArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.ArmRetract:
                return {
                    command: "retract arm",
                    callback: () => this.incrementalArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.WristRotateIn:
                return {
                    command: "rotate wrist counterclockwise",
                    callback: () => this.incrementalArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.WristRotateOut:
                return {
                    command: "rotate wrist clockwise",
                    callback: () => this.incrementalArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.GripperOpen:
                return {
                    command: "open gripper",
                    callback: () => this.incrementalArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.GripperClose:
                return {
                    command: "close gripper",
                    callback: () => this.incrementalArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale),
                }
            case VoiceCommandFunction.Stop:
                return {
                    command: "stop",
                    callback: () => this.stopCurrentAction()
                }
        }
    }
}