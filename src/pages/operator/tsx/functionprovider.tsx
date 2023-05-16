import React from 'react'
import { RemoteRobot } from "robot/tsx/remoterobot"
import { VelocityCommand } from 'utils/commands'
import { ActionMode } from "./actionmodebutton"
import { UserInteractionFunction, ButtonFunctionProps } from "./buttonpads"
import { JOINT_VELOCITIES, JOINT_INCREMENTS }from 'utils/util'

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
    }

    handleActionModeUpdate(newActionMode: ActionMode) {
        this.actionMode = newActionMode
    }
    
    handleVelocityScaleUpdate(newVelocityScale: number) {
        this.velocityScale = newVelocityScale
    }

    provideFunctions(interactionFn: UserInteractionFunction): ButtonFunctionProps {
        switch (this.actionMode) {
            case ActionMode.StepActions:
                switch (interactionFn) {
                    case UserInteractionFunction.BaseForward: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.driveBase(JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.activeVelocityAction?.stop() 
                        }
                    case UserInteractionFunction.BaseReverse: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.driveBase(-1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale, 0.0),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.BaseRotateLeft: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.driveBase(0.0, JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.BaseRotateRight: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.driveBase(0.0, -1 * JOINT_VELOCITIES["translate_mobile_base"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.ArmLower: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_lift", -1 * JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.ArmLift: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_lift", JOINT_INCREMENTS["joint_lift"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.ArmExtend: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("wrist_extension", JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.ArmRetract: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("wrist_extension", -1 * JOINT_INCREMENTS["wrist_extension"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.WristRotateIn: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_wrist_yaw", JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.WristRotateOut: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_wrist_yaw", -1 * JOINT_INCREMENTS["joint_wrist_yaw"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.GripperOpen: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_gripper_finger_left", JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                    case UserInteractionFunction.GripperOpen: 
                        return { 
                            onClick: () => this.activeVelocityAction = 
                                this.remoteRobot.incrementalMove("joint_gripper_finger_left", -1 * JOINT_INCREMENTS["joint_gripper_finger_left"]! * this.velocityScale),
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                }
                break;
            case ActionMode.PressRelease:
            case ActionMode.ClickClick:
        }

        // throw 'Functions do not exist for: ' + interactionFn
        return {
            onClick: () => console.log(`${this.actionMode} ${interactionFn} clicked`),
            onRelease: () => console.log(`${this.actionMode} ${interactionFn} releasd`)
        }
    }
}