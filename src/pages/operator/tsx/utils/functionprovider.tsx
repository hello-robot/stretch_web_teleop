import React from 'react'
import { RemoteRobot } from "shared/remoterobot"
import { VelocityCommand } from 'shared/commands'
import { ActionMode } from "../staticcomponents/actionmodebutton"
import { ButtonPadFunction, ButtonFunctions } from "../layoutcomponents/buttonpads"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints }from 'shared/util'
import { PredictiveDisplayFunctions } from '../layoutcomponents/predictivedisplay'

interface ButtonFunctionProviderProps {
    actionMode: ActionMode;
    velocityScale: number;
}

/**
 * Function that takes a button function enum and returns the
 * corresponding button function props.
 */
// export type FunctionProvider = (funct: UserInteractionFunction) => ButtonFunctions
export class FunctionProvider {
    protected static remoteRobot?: RemoteRobot;
    constructor(props: {}) {}

    static addRemoteRobot(remoteRobot: RemoteRobot) {
        FunctionProvider.remoteRobot = remoteRobot;
    }
}

export class ButtonFunctionProvider extends FunctionProvider {
    private activeVelocityAction?: VelocityCommand;
    private velocityExecutionHeartbeat?: number // ReturnType<typeof setInterval>

    private actionMode: ActionMode;
    private velocityScale: number;
    // private remoteRobot: RemoteRobot;

    constructor(props: ButtonFunctionProviderProps) {
        super(props)
        this.actionMode = props.actionMode,
        this.velocityScale = props.velocityScale
        // FunctionProvider.remoteRobot = props.remoteRobot
        this.provideFunctions = this.provideFunctions.bind(this)
        this.handleActionModeUpdate = this.handleActionModeUpdate.bind(this)
        this.handleVelocityScaleUpdate = this.handleVelocityScaleUpdate.bind(this)
    }

    handleActionModeUpdate(newActionMode: ActionMode) {
        console.log(newActionMode)
        this.actionMode = newActionMode
    }

    handleVelocityScaleUpdate(newVelocityScale: number) {
        this.velocityScale = newVelocityScale
    }

    incrementalBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
    }

    incrementalArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction = FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
    }

    continuousBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction = 
            FunctionProvider.remoteRobot?.driveBase(linVel, angVel),
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = 
                FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
        }, 150);
    }

    continuousArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction = 
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = 
                FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        }, 150);
    }

    stopCurrentAction() {
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

    provideFunctions(interactionFn: ButtonPadFunction): ButtonFunctions | PredictiveDisplayFunctions {
        switch (this.actionMode) {
            case ActionMode.StepActions:
                switch (interactionFn) {
                    case ButtonPadFunction.BaseForward:
                        return {
                            onClick: () => this.incrementalBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case ButtonPadFunction.BaseReverse:
                        return {
                            onClick: () => this.incrementalBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateLeft:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
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
                switch (interactionFn) {
                    case ButtonPadFunction.BaseForward: 
                        return { 
                            onClick: () => this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseReverse: 
                    return { 
                        onClick: () => this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                        onRelease: () => this.stopCurrentAction(), 
                        onLeave: () => this.stopCurrentAction()
                    }
                    case ButtonPadFunction.BaseRotateLeft: 
                        return { 
                            onClick: () => this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight: 
                        return { 
                            onClick: () => this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend: 
                        return { 
                            onClick: () => this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract: 
                        return { 
                            onClick: () => this.continuousArmMovement("wrist_extension", -1*JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", -1*JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
            case ActionMode.ClickClick:
                switch (interactionFn) {
                    case ButtonPadFunction.BaseForward: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseReverse: 
                    return { 
                        onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                            this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                        onLeave: () => this.stopCurrentAction()
                    }
                    case ButtonPadFunction.BaseRotateLeft: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.BaseRotateRight: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLift: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmLower: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmExtend: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.ArmRetract: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() : 
                                this.continuousArmMovement("wrist_extension", -1*JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateIn: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() : 
                                this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.WristRotateOut: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperOpen: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case ButtonPadFunction.GripperClose: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_gripper_finger_left", -1*JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
        }

        // throw 'Functions do not exist for: ' + interactionFn
        return {
            onClick: () => console.log(`${this.actionMode} ${interactionFn} clicked`),
            onRelease: () => console.log(`${this.actionMode} ${interactionFn} releasd`)
        }
    }
}