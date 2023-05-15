import { RemoteRobot } from "robot/tsx/remoterobot"
import { ActionMode } from "./actionmodebutton"
import { UserInteractionFunction, ButtonFunctionProps } from "./buttonpads"

/**
 * Function that takes a button function enum and returns the
 * corresponding button function props.
 */
export type FunctionProvider = (funct: UserInteractionFunction) => ButtonFunctionProps

export const mockFunctionProvider = (am: ActionMode, funct: UserInteractionFunction, remoteRobot: RemoteRobot): ButtonFunctionProps => {
    switch (am) {
        case ActionMode.StepActions:
            switch (funct) {
                case UserInteractionFunction.BaseForward: return { onClick: () => remoteRobot.driveWithVelocities(0.5, 0.0) }
            }
            break;
        case ActionMode.PressRelease:
        case ActionMode.ClickClick:
    }
    return {
        onClick: () => console.log(`${am} ${funct} clicked`),
        onRelease: () => console.log(`${am} ${funct} releasd`)
    }
}