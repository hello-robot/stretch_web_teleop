import React from 'react'
import { RemoteRobot } from "robot/tsx/remoterobot"
import { VelocityCommand } from 'utils/commands'
import { ActionMode } from "./actionmodebutton"
import { UserInteractionFunction, ButtonFunctionProps } from "./buttonpads"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints }from 'utils/util'

interface FunctionProviderState {
    actionMode: ActionMode
    velocityScale: number
}

interface FunctionProviderProps {
    actionMode: ActionMode;
    velocityScale: number;
    remoteRobot: RemoteRobot;
}

/**
 * Function that takes a button function enum and returns the
 * corresponding button function props.
 */
export type FunctionProvider = (funct: UserInteractionFunction) => ButtonFunctionProps

export class ButtonFunctionProvider extends React.Component<FunctionProviderProps, FunctionProviderState> {
    private activeVelocityAction?: VelocityCommand;
    private velocityExecutionHeartbeat?: number // ReturnType<typeof setInterval>

    private actionMode: ActionMode;
    private velocityScale: number;
    private remoteRobot: RemoteRobot;

    constructor(props: FunctionProviderProps) {
        super(props)
        this.actionMode = props.actionMode,
        this.velocityScale = props.velocityScale
        this.remoteRobot = props.remoteRobot
        this.provideFunctions = this.provideFunctions.bind(this)
        this.handleActionModeUpdate = this.handleActionModeUpdate.bind(this)
        this.handleVelocityScaleUpdate = this.handleVelocityScaleUpdate.bind(this)
        this.remoteRobot.setRobotMode("navigation")
    }

    handleActionModeUpdate(newActionMode: ActionMode) {
        console.log(newActionMode)
        this.actionMode = newActionMode
    }

    handleVelocityScaleUpdate(newVelocityScale: number) {
        this.velocityScale = newVelocityScale
    }

    incrementalBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction = this.remoteRobot.driveBase(linVel, angVel)
    }

    incrementalArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction = this.remoteRobot.incrementalMove(jointName, increment)
    }

    continuousBaseDrive(linVel: number, angVel: number) {
        this.activeVelocityAction = 
            this.remoteRobot.driveBase(linVel, angVel),
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = 
                this.remoteRobot!.driveBase(linVel, angVel)
        }, 150);
    }

    continuousArmMovement(jointName: ValidJoints, increment: number) {
        this.activeVelocityAction = 
            this.remoteRobot.incrementalMove(jointName, increment)
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction = 
                this.remoteRobot.incrementalMove(jointName, increment)
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

    provideFunctions(interactionFn: UserInteractionFunction): ButtonFunctionProps {
        switch (this.actionMode) {
            case ActionMode.StepActions:
                switch (interactionFn) {
                    case UserInteractionFunction.BaseForward:
                        return {
                            onClick: () => this.incrementalBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.BaseReverse:
                        return {
                            onClick: () => this.incrementalBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseRotateLeft:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseRotateRight:
                        return {
                            onClick: () => this.incrementalBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLower:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLift:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmExtend:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmRetract:
                        return {
                            onClick: () => this.incrementalArmMovement("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateIn:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateOut:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperOpen:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperClose:
                        return {
                            onClick: () => this.incrementalArmMovement("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
            case ActionMode.PressRelease:
                switch (interactionFn) {
                    case UserInteractionFunction.BaseForward: 
                        return { 
                            onClick: () => this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseReverse: 
                    return { 
                        onClick: () => this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                        onRelease: () => this.stopCurrentAction(), 
                        onLeave: () => this.stopCurrentAction()
                    }
                    case UserInteractionFunction.BaseRotateLeft: 
                        return { 
                            onClick: () => this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseRotateRight: 
                        return { 
                            onClick: () => this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLower: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLift: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmExtend: 
                        return { 
                            onClick: () => this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmRetract: 
                        return { 
                            onClick: () => this.continuousArmMovement("wrist_extension", -1*JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateIn: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateOut: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperOpen: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperClose: 
                        return { 
                            onClick: () => this.continuousArmMovement("joint_gripper_finger_left", -1*JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onRelease: () => this.stopCurrentAction(), 
                            onLeave: () => this.stopCurrentAction()
                        }
                }
                break;
            case ActionMode.ClickClick:
                switch (interactionFn) {
                    case UserInteractionFunction.BaseForward: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseReverse: 
                    return { 
                        onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                            this.continuousBaseDrive(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                        onLeave: () => this.stopCurrentAction()
                    }
                    case UserInteractionFunction.BaseRotateLeft: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.BaseRotateRight: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousBaseDrive(0.0, -1 * JOINT_VELOCITIES["rotate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLift: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmLower: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmExtend: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.ArmRetract: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() : 
                                this.continuousArmMovement("wrist_extension", -1*JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateIn: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() : 
                                this.continuousArmMovement("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.WristRotateOut: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperOpen: 
                        return { 
                            onClick: () => this.activeVelocityAction ? this.stopCurrentAction() :
                                this.continuousArmMovement("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.stopCurrentAction()
                        }
                    case UserInteractionFunction.GripperClose: 
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