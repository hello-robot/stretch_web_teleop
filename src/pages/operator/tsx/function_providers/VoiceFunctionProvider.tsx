import { FunctionProvider } from "./FunctionProvider"
import { VoiceCommandFunction, VoiceCommandFunctions, VoiceCommandsProps } from "../static_components/VoiceCommands"
import { JOINT_VELOCITIES, JOINT_INCREMENTS } from 'shared/util'
import { VELOCITY_SCALE } from "../static_components/SpeedControl"
import { ActionMode, LayoutGridDefinition, ComponentType, CameraViewId, CameraViewDefinition, RealsenseVideoStreamDef, AdjustableOverheadVideoStreamDef } from "../utils/component_definitions";
import { buttonFunctionProvider, underVideoFunctionProvider } from "..";
import { ButtonFunctions, ButtonPadButton, ButtonState } from "./ButtonFunctionProvider";
import { UnderVideoButton } from "./UnderVideoFunctionProvider";

export class VoiceFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public stopCurrentAction() {
        let activeButtonMap: [ButtonPadButton, ButtonState] | undefined = buttonFunctionProvider.getActiveButton()
        let activeButton: ButtonPadButton = activeButtonMap ? activeButtonMap[0] : undefined
        if (activeButton !== undefined) {
            switch(FunctionProvider.actionMode) {
                case ActionMode.StepActions:
                    this.stopCurrentAction()
                    break;
                case ActionMode.PressAndHold:
                    buttonFunctionProvider.provideFunctions(activeButton).onRelease()
                    break;
                case ActionMode.ClickClick:
                    buttonFunctionProvider.provideFunctions(activeButton).onClick()
                    break;
            }
        }
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
        props: VoiceCommandsProps,
        handleCommand: (command: string) => void): VoiceCommandFunctions 
    {
        switch (voiceCommandFunction) {
            case VoiceCommandFunction.BaseForward:
                return {
                    command: "drive forward",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseForward).onClick()
                        handleCommand("Drive forward")
                    }
                }
            case VoiceCommandFunction.BaseReverse:
                return {
                    command: "drive backward",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseReverse).onClick()
                        handleCommand("Drive backward")
                    },
                }
            case VoiceCommandFunction.BaseRotateLeft:
                return {
                    command: "turn left",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseRotateLeft).onClick()
                        handleCommand("Turn left")
                    },
                }
            case VoiceCommandFunction.BaseRotateRight:
                return {
                    command: "turn right",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseRotateRight).onClick()
                        handleCommand("Turn right")
                    },
                }
            case VoiceCommandFunction.ArmLower:
                return {
                    command: "lower arm",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLower).onClick()
                        handleCommand("Lower arm")
                    },
                }
            case VoiceCommandFunction.ArmLift:
                return {
                    command: "raise arm",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLift).onClick()
                        handleCommand("Raise arm")
                    },
                }
            case VoiceCommandFunction.ArmExtend:
                return {
                    command: "extend arm",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmExtend).onClick()
                        handleCommand("Extend arm")
                    },
                }
            case VoiceCommandFunction.ArmRetract:
                return {
                    command: "retract arm",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmRetract).onClick()
                        handleCommand("Retract arm")
                    },
                }
            case VoiceCommandFunction.WristRotateIn:
                return {
                    command: "rotate left",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRotateIn).onClick()
                        handleCommand("Rotate left")
                    },
                }
            case VoiceCommandFunction.WristRotateOut:
                return {
                    command: "rotate right",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRotateOut).onClick()
                        handleCommand("Rotate right")
                    },
                }
            case VoiceCommandFunction.WristPitchUp:
                return {
                    command: "pitch up",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristPitchUp).onClick()
                        handleCommand("Pitch up")
                    },
                }
            case VoiceCommandFunction.WristPitchDown:
                return {
                    command: "pitch down",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristPitchDown).onClick()
                        handleCommand("Pitch down")
                    },
                }
            case VoiceCommandFunction.WristRollLeft:
                return {
                    command: "roll left",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRollLeft).onClick()
                        handleCommand("Roll left")
                    },
                }
            case VoiceCommandFunction.WristRollRight:
                return {
                    command: "roll right",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRollRight).onClick()
                        handleCommand("Roll right")
                    },
                }
            case VoiceCommandFunction.GripperOpen:
                return {
                    command: "open gripper",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.GripperOpen).onClick()
                        handleCommand("Open gripper")
                    },
                }
            case VoiceCommandFunction.GripperClose:
                return {
                    command: "close gripper",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.GripperClose).onClick()
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
                        props.onUpdateVelocityScale(VELOCITY_SCALE[speeds.indexOf(speed as string)].scale)
                        handleCommand(`Set speed to ${speed}`)
                    }
                }
            case VoiceCommandFunction.SetActionMode:
                return {
                    command: "set action mode to *",
                    callback: (actionMode) => {
                        const actionModes = ["step actions", "press release", "click click"]
                        const modes = Object.values(ActionMode)
                        if (!actionModes.includes(actionMode as string)) return;
                        props.onUpdateActionMode(modes[actionModes.indexOf(actionMode as string)])
                        handleCommand(`Set action mode to ${actionMode}`)
                    }
                }
            case VoiceCommandFunction.FollowGripper:
                return {
                    command: "* follow gripper",
                    callback: (action) => {
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action as string)) return;
                        const toggle = action == "check" ? true : false
                        underVideoFunctionProvider.provideFunctions(UnderVideoButton.FollowGripper).onCheck(toggle)
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.FollowGripper)
                        handleCommand(`${action}ed follow gripper`)
                    }
                }
            case VoiceCommandFunction.PredictiveDisplay:
                return {
                    command: "* predictive display",
                    callback: (action) => {
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action as string)) return;
                        const toggle = action == "check" ? true : false
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.PredictiveDisplay)
                        handleCommand(`${action}ed predictive display`)
                    }
                }
            case VoiceCommandFunction.RealsenseDepthSensing:
                return {
                    command: "* depth sensing",
                    callback: (action) => {
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action as string)) return;
                        const toggle = action == "check" ? true : false
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.RealsenseDepthSensing)
                        handleCommand(`${action}ed realsense depth sensing`)
                    }
                }
            case VoiceCommandFunction.SelectObject:
                return {
                    command: "* select object",
                    callback: (action) => {
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action as string)) return;
                        const toggle = action == "check" ? true : false
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.SelectObject)
                        handleCommand(`${action}ed select object`)
                    }
                }
        }
    }
}