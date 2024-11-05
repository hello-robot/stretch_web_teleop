import { FunctionProvider } from "./FunctionProvider"
import { Speed, VoiceCommandFunction, VoiceCommandFunctions, VoiceCommandsProps } from "../static_components/VoiceCommands"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, wordsToNumbers } from 'shared/util'
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
        arg: Speed | ActionMode,
        props: VoiceCommandsProps,
        // handleCommand: (command: string) => void
        ): () => void 
    {
        switch (voiceCommandFunction) {
            case VoiceCommandFunction.SetSpeed:
                return () => {
                    props.onUpdateVelocityScale(VELOCITY_SCALE[arg as Speed].scale)
                }
            case VoiceCommandFunction.SetActionMode:
                return () => {
                    props.onUpdateActionMode(arg as ActionMode)
                }
            case VoiceCommandFunction.BaseForward:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseForward).onClick()
                }
            case VoiceCommandFunction.BaseReverse:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseReverse).onClick()
                }
            case VoiceCommandFunction.BaseRotateLeft:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseRotateLeft).onClick()
                }
            case VoiceCommandFunction.BaseRotateRight:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.BaseRotateRight).onClick()
                }
            case VoiceCommandFunction.ArmLower:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLower).onClick()
                }
            case VoiceCommandFunction.ArmLift:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmLift).onClick()
                }
            case VoiceCommandFunction.ArmExtend:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmExtend).onClick()
                }
            case VoiceCommandFunction.ArmRetract:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.ArmRetract).onClick()
                }
            case VoiceCommandFunction.WristRotateIn:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRotateIn).onClick()
                }
            case VoiceCommandFunction.WristRotateOut:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRotateOut).onClick()
                }
            case VoiceCommandFunction.WristPitchUp:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristPitchUp).onClick()
                }
            case VoiceCommandFunction.WristPitchDown:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristPitchDown).onClick()
                }
            case VoiceCommandFunction.WristRollLeft:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRollLeft).onClick()
                }
            case VoiceCommandFunction.WristRollRight:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.WristRollRight).onClick()
                }
            case VoiceCommandFunction.GripperOpen:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.GripperOpen).onClick()
                }
            case VoiceCommandFunction.GripperClose:
                return () => {
                    this.stopCurrentAction()
                    buttonFunctionProvider.provideFunctions(ButtonPadButton.GripperClose).onClick()
                }
            case VoiceCommandFunction.Stop:
                return () => {
                        this.stopCurrentAction()
                }
        //     case VoiceCommandFunction.FollowGripper:
        //         return {
        //             command: "* follow gripper",
        //             callback: (action) => {
        //                 action = action.toLocaleLowerCase()
        //                 const actions = ["check", "uncheck"]
        //                 if (!actions.includes(action as string)) return;
        //                 const toggle = action == "check" ? true : false
        //                 underVideoFunctionProvider.provideFunctions(UnderVideoButton.FollowGripper).onCheck(toggle)
        //                 props.onToggleUnderVideoButtons(toggle, UnderVideoButton.FollowGripper)
        //                 handleCommand(`${action}ed follow gripper`)
        //             }
        //         }
        //     case VoiceCommandFunction.PredictiveDisplay:
        //         return {
        //             command: "* predictive display",
        //             callback: (action) => {
        //                 action = action.toLocaleLowerCase()
        //                 const actions = ["check", "uncheck"]
        //                 if (!actions.includes(action as string)) return;
        //                 const toggle = action == "check" ? true : false
        //                 props.onToggleUnderVideoButtons(toggle, UnderVideoButton.PredictiveDisplay)
        //                 handleCommand(`${action}ed predictive display`)
        //             }
        //         }
        //     case VoiceCommandFunction.RealsenseDepthSensing:
        //         return {
        //             command: "* depth sensing",
        //             callback: (action) => {
        //                 action = action.toLocaleLowerCase()
        //                 const actions = ["check", "uncheck"]
        //                 if (!actions.includes(action as string)) return;
        //                 const toggle = action == "check" ? true : false
        //                 props.onToggleUnderVideoButtons(toggle, UnderVideoButton.RealsenseDepthSensing)
        //                 handleCommand(`${action}ed realsense depth sensing`)
        //             }
        //         }
        //     case VoiceCommandFunction.SelectObject:
        //         return {
        //             command: "* select object",
        //             callback: (action) => {
        //                 action = action.toLocaleLowerCase()
        //                 const actions = ["check", "uncheck"]
        //                 if (!actions.includes(action as string)) return;
        //                 const toggle = action == "check" ? true : false
        //                 props.onToggleUnderVideoButtons(toggle, UnderVideoButton.SelectObject)
        //                 handleCommand(`${action}ed select object`)
        //             }
        //         }
        //     case VoiceCommandFunction.DetectObjects:
        //         return {
        //             command: "* detect objects",
        //             callback: (action) => {
        //                 action = action.toLocaleLowerCase()
        //                 const actions = ["check", "uncheck"]
        //                 if (!actions.includes(action.toLowerCase() as string)) return;
        //                 const toggle = action == "check" ? true : false
        //                 props.onToggleUnderVideoButtons(toggle, UnderVideoButton.DetectObjects)
        //                 underVideoFunctionProvider.provideFunctions(
        //                     UnderVideoButton.DetectObjects,
        //                 ).onCheck(toggle);
        //                 handleCommand(`${action}ed detect objects`)
        //             }
        //         }
        //     case VoiceCommandFunction.SelectDetectedObject:
        //         return {
        //             command: "select object number *",
        //             callback: (objNumber) => {
        //                 let objNum = wordsToNumbers(objNumber)
        //                 underVideoFunctionProvider.provideFunctions(
        //                     UnderVideoButton.VoiceSelectObject,
        //                 ).set(objNum);
        //                 handleCommand(`Selected object number ${objNum}`)
        //             }
        //         }
        //     case VoiceCommandFunction.HorizontalGrasp:
        //         return {
        //             command: "click horizontal",
        //             callback: () => {
        //                 underVideoFunctionProvider.provideFunctions(
        //                     UnderVideoButton.VoiceMoveToPregraspHorizontal
        //                 ).onClick()
        //                 handleCommand(`Moving to horizontal pregrasp position`)
        //             }
        //         }
        //     case VoiceCommandFunction.VerticalGrasp:
        //         return {
        //             command: "click vertical",
        //             callback: () => {
        //                 underVideoFunctionProvider.provideFunctions(
        //                     UnderVideoButton.VoiceMoveToPregraspVertical
        //                 ).onClick()
        //                 handleCommand(`Moving to vertical pregrasp position`)
        //             }
        //         }
        }
    }
}