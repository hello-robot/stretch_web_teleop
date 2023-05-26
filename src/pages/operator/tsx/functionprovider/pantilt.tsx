import { FunctionProvider } from "./functionprovider"
import { PanTiltButtonDirection, PanTitleButtonFunctions } from "../layoutcomponents/videostreamcomponent"
import{ JOINT_INCREMENTS } from 'shared/util'

export class PantiltFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public provideFunctions(direction: PanTiltButtonDirection): PanTitleButtonFunctions {
        switch (direction) {
            case 'up':
                console.log("up")
                return {
                    onClick: () => this.incrementalJointMovement('joint_head_tilt',  JOINT_INCREMENTS["joint_head_tilt"]! * FunctionProvider.velocityScale)
                }
            case 'down':
                console.log("down")
                return {
                    onClick: () => this.incrementalJointMovement('joint_head_tilt',  -1 * JOINT_INCREMENTS["joint_head_tilt"]! * FunctionProvider.velocityScale)
                }
            case 'left':
                console.log("left")
                return {
                    onClick: () => this.incrementalJointMovement('joint_head_pan',  JOINT_INCREMENTS["joint_head_pan"]! * FunctionProvider.velocityScale)
                }
            case 'right':
                console.log("right")
                return {
                    onClick: () => this.incrementalJointMovement('joint_head_pan',  -1 * JOINT_INCREMENTS["joint_head_pan"]! * FunctionProvider.velocityScale)
                }
        }
    }
}