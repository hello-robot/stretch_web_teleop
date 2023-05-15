import React from 'react'
import { RemoteRobot } from "robot/tsx/remoterobot"
import { VelocityCommand } from 'utils/commands'
import { ActionMode } from "./actionmodebutton"
import { UserInteractionFunction, ButtonFunctionProps } from "./buttonpads"

interface FunctionProviderState {
    actionMode: ActionMode
}

interface FunctionProviderProps {
    actionMode: ActionMode;
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

    private remoteRobot: RemoteRobot;
    
    constructor(props: FunctionProviderProps) {
        super(props)
        this.state = {
            actionMode: props.actionMode
        }
        this.remoteRobot = props.remoteRobot
    }

    handleActionModeUpdate(newActionMode: ActionMode) {
        this.setState({ actionMode: newActionMode })
    }
    
    provideFunctions(interactionFn: UserInteractionFunction): ButtonFunctionProps {
        const { actionMode } = this.state
        switch (actionMode) {
            case ActionMode.StepActions:
                switch (interactionFn) {
                    case UserInteractionFunction.BaseForward: 
                        return { 
                            onClick: () => this.activeVelocityAction = this.remoteRobot.driveBase(0.5, 0.0) ,
                            onLeave: () => this.activeVelocityAction?.stop()
                        }
                }
                break;
            case ActionMode.PressRelease:
            case ActionMode.ClickClick:
        }

        // throw 'Functions do not exist for: ' + interactionFn
        return {
            onClick: () => console.log(`${actionMode} ${interactionFn} clicked`),
            onRelease: () => console.log(`${actionMode} ${interactionFn} releasd`)
        }
    }
}