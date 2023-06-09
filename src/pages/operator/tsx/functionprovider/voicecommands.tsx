import { FunctionProvider } from "./functionprovider"
import { VoiceCommandFunction, VoiceCommandFunctions } from "../staticcomponents/VoiceCommands"
import { JOINT_VELOCITIES, JOINT_INCREMENTS } from 'shared/util'
import { VELOCITY_SCALE } from "../staticcomponents/VelocityControl"

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
    public provideFunctions(
        voiceCommandFunction: VoiceCommandFunction, 
        onUpdateVelocityScale: (newScale: number) => void,
        handleCommand: (command: string) => void): VoiceCommandFunctions 
    {
        switch (voiceCommandFunction) {
            case VoiceCommandFunction.BaseForward:
                return {
                    command: "drive forward",
                    callback: () => {
                        this.incrementalBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0)
                        handleCommand("Drive forward")
                    }
                }
            case VoiceCommandFunction.BaseReverse:
                return {
                    command: "drive backward",
                    callback: () => { 
                        this.incrementalBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale, 0.0)
                        handleCommand("Drive backward")
                    },
                }
            case VoiceCommandFunction.BaseRotateLeft:
                return {
                    command: "rotate robot left",
                    callback: () => {
                        this.incrementalBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale)
                        handleCommand("Rotate robot left")
                    },
                }
            case VoiceCommandFunction.BaseRotateRight:
                return {
                    command: "rotate robot right",
                    callback: () => {
                        this.incrementalBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale)
                        handleCommand("Rotate robot right")
                    },
                }
            case VoiceCommandFunction.ArmLower:
                return {
                    command: "lower arm",
                    callback: () => {
                        this.incrementalJointMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale)
                        handleCommand("Lower arm")
                    },
                }
            case VoiceCommandFunction.ArmLift:
                return {
                    command: "raise arm",
                    callback: () => {
                        this.incrementalJointMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * FunctionProvider.velocityScale)
                        handleCommand("Raise arm")
                    },
                }
            case VoiceCommandFunction.ArmExtend:
                return {
                    command: "extend arm",
                    callback: () => {
                        this.incrementalJointMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale)
                        handleCommand("Extend arm")
                    },
                }
            case VoiceCommandFunction.ArmRetract:
                return {
                    command: "retract arm",
                    callback: () => {
                        this.incrementalJointMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * FunctionProvider.velocityScale)
                        handleCommand("Retract arm")
                    },
                }
            case VoiceCommandFunction.WristRotateIn:
                return {
                    command: "rotate wrist counterclockwise",
                    callback: () => {
                        this.incrementalJointMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale)
                        handleCommand("Rotate wrist counterclockwise")
                    },
                }
            case VoiceCommandFunction.WristRotateOut:
                return {
                    command: "rotate wrist clockwise",
                    callback: () => {
                        this.incrementalJointMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * FunctionProvider.velocityScale)
                        handleCommand("Rotate wrist clockwise")
                    },
                }
            case VoiceCommandFunction.GripperOpen:
                return {
                    command: "open gripper",
                    callback: () => {
                        this.incrementalJointMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale)
                        handleCommand("Open gripper")
                    },
                }
            case VoiceCommandFunction.GripperClose:
                return {
                    command: "close gripper",
                    callback: () => {
                        this.incrementalJointMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * FunctionProvider.velocityScale)
                        handleCommand("Close gripper")
                    },
                }
            case VoiceCommandFunction.Stop:
                return {
                    command: "stop",
                    callback: () => {
                        this.stopCurrentAction()
                        handleCommand("Stop")
                    }
                }
            case VoiceCommandFunction.SetSpeed:
                return {
                    command: "set speed to *",
                    callback: (speed) => {
                        const speeds = ["slowest", "slow", "medium", "fast", "fastest"]
                        if (!speeds.includes(speed as string)) return;
                        onUpdateVelocityScale(VELOCITY_SCALE[speeds.indexOf(speed as string)].scale)
                        handleCommand(`Set speed to ${speed}`)
                    }
                }
        }
    }
}