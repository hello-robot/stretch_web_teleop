import { ButtonPadFunction, ButtonFunctions } from "../layoutcomponents/buttonpads"
import { FunctionProvider } from 'operator/tsx/functionprovider/functionprovider'
import { ActionMode } from "operator/tsx/staticcomponents/actionmodebutton"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints } from 'shared/util'

/**
 * Provides functions for the button pads
 */
 export class ButtonFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    /**
     * Takes a ButtonPadFunction which indicates the type of button (e.g. drive 
     * base forward, lift arm), and returns a set of functions to execute when 
     * the user interacts with the button.
     * 
     * @param buttonPadFunction the {@link ButtonPadFunction}
     * @returns the {@link ButtonFunctions} for the button
     */
    public provideFunctions(buttonPadFunction: ButtonPadFunction): ButtonFunctions {
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
    }
}
