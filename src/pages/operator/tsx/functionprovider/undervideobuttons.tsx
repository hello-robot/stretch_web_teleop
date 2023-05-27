import { FunctionProvider } from "./functionprovider"

export enum UnderVideoButton {
    DriveView = "Drive View",
    GripperView = "Gripper View",
    LookAtGripper = "Look At Gripper",
    LookAtBase = "Look At Base",
    FollowGripper = "Follow Gripper"
}

/** Array of different views for the overhead camera */
export const overheadButtons: UnderVideoButton[] = [UnderVideoButton.DriveView, UnderVideoButton.GripperView]
/** Type to specify the different overhead camera views */
export type OverheadButtons = typeof overheadButtons[number]

export type UnderVideoButtonFunctions = {
    onClick: () => void
}

export class UnderVideoFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public provideFunctions(button: UnderVideoButton): UnderVideoButtonFunctions {
        switch (button) {
            case UnderVideoButton.DriveView:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setCameraPerspective("overhead", "nav") 
                }
            case UnderVideoButton.GripperView:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setCameraPerspective("overhead", "manip") 
                }
        }
    }
}