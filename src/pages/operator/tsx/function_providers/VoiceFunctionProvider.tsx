import { FunctionProvider } from "./FunctionProvider"
import { VoiceCommandFunction, VoiceCommandFunctions, VoiceCommandsProps } from "../static_components/VoiceCommands"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, wordsToNumbers, FEEDING_CONFIGURATION, STOW_WRIST_GRIPPER, STOW_ARM_WRIST_GRIPPER } from 'shared/util'
import { VELOCITY_SCALE } from "../static_components/SpeedControl"
import { ActionMode, LayoutGridDefinition, ComponentType, CameraViewId, CameraViewDefinition, RealsenseVideoStreamDef, AdjustableOverheadVideoStreamDef } from "../utils/component_definitions";
import { buttonFunctionProvider, underMapFunctionProvider, underVideoFunctionProvider } from "..";
import { ButtonFunctions, ButtonPadButton, ButtonState } from "./ButtonFunctionProvider";
import { UnderVideoButton } from "./UnderVideoFunctionProvider";
import { UnderMapButton } from "./UnderMapFunctionProvider";

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
                    command: "forward",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseForward).onClick()
                        handleCommand("Driving forward")
                    }
                }
            case VoiceCommandFunction.BaseReverse:
                return {
                    command: "backward",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseReverse).onClick()
                        handleCommand("Driving backward")
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
                    command: "arm down",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLower).onClick()
                        handleCommand("Arm down")
                    },
                }
            case VoiceCommandFunction.ArmLift:
                return {
                    command: "arm up",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLift).onClick()
                        handleCommand("Arm up")
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
            case VoiceCommandFunction.GripperClose:
                return {
                    command: "close gripper",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.GripperClose).onClick()
                        handleCommand("Close gripper")
                    },
                }
            case VoiceCommandFunction.PanLeft:
                return {
                    command: "look left",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.CameraPanLeft).onClick()
                        handleCommand("Look left")
                    },
                }
            case VoiceCommandFunction.PanRight:
                return {
                    command: "look right",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.CameraPanRight).onClick()
                        handleCommand("Look right")
                    },
                }
            case VoiceCommandFunction.TiltUp:
                return {
                    command: "look up",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.CameraTiltUp).onClick()
                        handleCommand("tilt up")
                    },
                }
            case VoiceCommandFunction.TiltDown:
                return {
                    command: "look down",
                    callback: () => {
                        this.stopCurrentAction()
                        buttonFunctionProvider.provideFunctions(ButtonPadButton.CameraTiltDown).onClick()
                        handleCommand("tilt down")
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
                    command: "*",
                    callback: (speed) => {
                        speed = speed.toLocaleLowerCase()
                        const speeds = ["slowest", "slow", "medium", "fast", "fastest"]
                        if (!speeds.includes(speed as string)) return;
                        props.onUpdateVelocityScale(VELOCITY_SCALE[speeds.indexOf(speed as string)].scale)
                        handleCommand(`Set speed to ${speed}`)
                    }
                }
            case VoiceCommandFunction.SetActionMode:
                return {
                    command: "*",
                    callback: (actionMode) => {
                        actionMode = actionMode.toLocaleLowerCase()
                        const actionModes = ["step actions", "press and hold", "click click"]
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
                        action = action.toLocaleLowerCase()
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
                        action = action.toLocaleLowerCase()
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
                        action = action.toLocaleLowerCase()
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
                        action = action.toLocaleLowerCase()
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action as string)) return;
                        const toggle = action == "check" ? true : false
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.SelectObject)
                        handleCommand(`${action}ed select object`)
                    }
                }
            case VoiceCommandFunction.DetectObjects:
                return {
                    command: "* detect objects",
                    callback: (action) => {
                        action = action.toLocaleLowerCase()
                        const actions = ["check", "uncheck"]
                        if (!actions.includes(action.toLowerCase() as string)) return;
                        const toggle = action == "check" ? true : false
                        props.onToggleUnderVideoButtons(toggle, UnderVideoButton.DetectObjects)
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.DetectObjects,
                        ).onCheck(toggle);
                        handleCommand(`${action}ed detect objects`)
                    }
                }
            case VoiceCommandFunction.SelectDetectedObject:
                return {
                    command: "number *",
                    callback: (objNumber) => {
                        let objNum = wordsToNumbers(objNumber)
                        console.log(objNum)
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.VoiceSelectObject,
                        ).set(objNum);
                        handleCommand(`Selected object ${objNum}`)
                    }
                }
            case VoiceCommandFunction.HorizontalGrasp:
                return {
                    command: "click horizontal",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.VoiceMoveToPregraspHorizontal
                        ).onClick()
                        handleCommand(`Moving to horizontal pregrasp position`)
                    }
                }
            case VoiceCommandFunction.VerticalGrasp:
                return {
                    command: "click vertical",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.VoiceMoveToPregraspVertical
                        ).onClick()
                        handleCommand(`Moving to vertical pregrasp position`)
                    }
                }
            case VoiceCommandFunction.LookAhead:
                return {
                    command: "look ahead",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.LookAhead
                        ).onClick()
                        handleCommand(`Look Ahead`)
                    }
                }
            case VoiceCommandFunction.LookAtBase:
                return {
                    command: "look at base",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.LookAtBase
                        ).onClick()
                        handleCommand(`Look at Base`)
                    }
                }
            case VoiceCommandFunction.LookAtGripper:
                return {
                    command: "look at gripper",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.LookAtGripper
                        ).onClick()
                        handleCommand(`Look at Gripper`)
                    }
                }
            case VoiceCommandFunction.CenterWrist:
                return {
                    command: "center wrist",
                    callback: () => {
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.CenterWrist
                        ).onClick()
                        handleCommand(`Center Wrist`)
                    }
                }
            case VoiceCommandFunction.FeedingMode:
                return {
                    command: "feeding mode",
                    callback: () => {
                        FunctionProvider.remoteRobot?.setRobotPose(
                            FEEDING_CONFIGURATION,
                        )
                        FunctionProvider.remoteRobot.setToggle("setFeedingMode", true)
                        handleCommand(`Setting feeding mode`)
                    }
                }
            case VoiceCommandFunction.DefaultMode:
                return {
                    command: "default mode",
                    callback: () => {
                        FunctionProvider.remoteRobot?.setRobotPose(
                            STOW_WRIST_GRIPPER,
                        )
                        FunctionProvider.remoteRobot.setToggle("setFeedingMode", false)
                        handleCommand(`Setting default mode`)
                    }
                }
            case VoiceCommandFunction.SelectBite:
                return {
                    command: "grab number *",
                    callback: (objNumber) => {
                        let objNum = wordsToNumbers(objNumber)
                        underVideoFunctionProvider.provideFunctions(
                            UnderVideoButton.VoiceSelectBite,
                        ).set(objNum);
                        handleCommand(`Selected object ${objNum}`)
                    }
                }
            case VoiceCommandFunction.GoTo:
                return {
                    command: "go to *",
                    callback: (location) => {
                        let getPoseNames = underMapFunctionProvider.provideFunctions(
                            UnderMapButton.GetSavedPoseNames
                        ) as () => string[]
                        let poseNames = getPoseNames().map(str => str.toLowerCase())
                        if (poseNames.includes(location)) {
                            let poseIdx: number = poseNames.indexOf(location)
                            underMapFunctionProvider.provideFunctions(UnderMapButton.LoadGoal)(poseIdx)
                            handleCommand(`Going to ${location}`)
                        }
                    }
                }
            case VoiceCommandFunction.Cancel:
                return {
                    command: "cancel",
                    callback: () => {
                        let cancelFn = underMapFunctionProvider.provideFunctions(
                            UnderMapButton.CancelGoal
                        ) as () => void
                        cancelFn()
                        handleCommand(`Cancelling navigation`)
                    }
                }
            case VoiceCommandFunction.StowArm:
                return {
                    command: "tuck arm",
                    callback: () => {
                        FunctionProvider.remoteRobot?.setRobotPose(
                            STOW_ARM_WRIST_GRIPPER,
                        )
                        handleCommand('Stowing arm')
                    }
                }
        }
    }
}