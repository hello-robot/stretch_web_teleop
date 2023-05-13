import { ActionMode } from "./actionmodebutton"
import { UserInteractionFunction, ButtonFunctionProps } from "./buttonpads"

/**
 * Function that takes a button function enum and returns the
 * corresponding button function props.
 */
export type FunctionProvider = (funct: UserInteractionFunction) => ButtonFunctionProps

export const mockFunctionProvider = (am: ActionMode, funct: UserInteractionFunction): ButtonFunctionProps => {
    return {
        onClick: () => console.log(`${am} ${funct} clicked`),
        onRelease: () => console.log(`${am} ${funct} releasd`)
    }
}