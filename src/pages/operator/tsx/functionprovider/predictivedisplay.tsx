import { FunctionProvider } from "./functionprovider"
import { PredictiveDisplayFunctions } from "../layoutcomponents/predictivedisplay"
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints } from 'shared/util'
import { ActionMode } from "../utils/componentdefinitions"

export class PredictiveDisplayFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    /**
     * Returns a set of functions to execute when 
     * the user interacts with predictive display mode
     * 
     * @returns the {@link PredictiveDisplayFunctions} for the action modes
     */
     public provideFunctions(): PredictiveDisplayFunctions {
        switch (FunctionProvider.actionMode) {
            case ActionMode.StepActions:
                return {
                    onClick: (length: number, angle: number) => 
                        this.incrementalBaseDrive(
                            JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale * length, 
                            JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale * angle
                        ),
                    onLeave: () => this.stopCurrentAction()
                }
            case ActionMode.PressRelease:
                return {
                    onClick: (length: number, angle: number) => 
                        this.continuousBaseDrive(
                            JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale * length, 
                            JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale * angle
                        ),
                    onMove: (length: number, angle: number) => 
                        this.activeVelocityAction ? this.continuousBaseDrive(
                            JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale * length, 
                            JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale * angle
                        ) : null,
                    onRelease: () => { this.stopCurrentAction(); console.log("on release") },
                    onLeave: () => this.stopCurrentAction()
                }
            case ActionMode.ClickClick:
                return {
                    onClick: (length: number, angle: number) => 
                        !this.activeVelocityAction ? this.continuousBaseDrive(
                            JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale * length, 
                            JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale * angle
                        ) : this.stopCurrentAction(),
                    onMove: (length: number, angle: number) => 
                        this.activeVelocityAction ? this.continuousBaseDrive(
                            JOINT_VELOCITIES["translate_mobile_base"]! * FunctionProvider.velocityScale * length, 
                            JOINT_VELOCITIES["rotate_mobile_base"]! * FunctionProvider.velocityScale * angle
                        ) : null,
                    onLeave: () => this.stopCurrentAction()
                }
        }
     }    
}