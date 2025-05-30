import { FunctionProvider } from "./FunctionProvider";
import { PredictiveDisplayFunctions } from "../layout_components/PredictiveDisplay";
import { JOINT_VELOCITIES, JOINT_INCREMENTS, ValidJoints } from "shared/util";
import { ActionModeType } from "../utils/component_definitions";

export class PredictiveDisplayFunctionProvider extends FunctionProvider {
    constructor() {
        super();
        this.provideFunctions = this.provideFunctions.bind(this);
    }

    /**
     * Returns a set of functions to execute when
     * the user interacts with predictive display mode
     *
     * @returns the {@link PredictiveDisplayFunctions} for the action modes
     */
    public provideFunctions(
        setActiveCallback: (active: boolean) => void,
    ): PredictiveDisplayFunctions {
        const baseLinVel =
            JOINT_VELOCITIES["translate_mobile_base"]! *
            FunctionProvider.velocityScale;
        const baseAngVel =
            JOINT_VELOCITIES["rotate_mobile_base"]! *
            FunctionProvider.velocityScale;
        switch (FunctionProvider.actionMode) {
            case ActionModeType.StepActions:
                return {
                    onClick: (length: number, angle: number) => {
                        this.incrementalBaseDrive(
                            baseLinVel * length,
                            baseAngVel * angle,
                        );
                        setActiveCallback(true);
                        setTimeout(() => setActiveCallback(false), 1000);
                    },
                    onLeave: () => {
                        this.stopCurrentAction();
                        setActiveCallback(false);
                    },
                };
            case ActionModeType.PressAndHold:
                return {
                    onClick: (length: number, angle: number) => {
                        this.continuousBaseDrive(
                            baseLinVel * length,
                            baseAngVel * angle,
                        );
                        setActiveCallback(true);
                    },
                    onMove: (length: number, angle: number) =>
                        this.activeVelocityAction
                            ? this.continuousBaseDrive(
                                baseLinVel * length,
                                baseAngVel * angle,
                            )
                            : null,
                    onRelease: () => {
                        this.stopCurrentAction();
                        setActiveCallback(false);
                    },
                    onLeave: () => {
                        this.stopCurrentAction();
                        setActiveCallback(false);
                    },
                };
            case ActionModeType.ClickClick:
                return {
                    onClick: (length: number, angle: number) => {
                        if (this.activeVelocityAction) {
                            this.stopCurrentAction();
                            setActiveCallback(false);
                        } else {
                            this.continuousBaseDrive(
                                baseLinVel * length,
                                baseAngVel * angle,
                            );
                            setActiveCallback(true);
                        }
                    },
                    onMove: (length: number, angle: number) => {
                        if (this.activeVelocityAction) {
                            this.continuousBaseDrive(
                                baseLinVel * length,
                                baseAngVel * angle,
                            );
                        }
                    },
                    onLeave: () => {
                        this.stopCurrentAction();
                        setActiveCallback(false);
                    },
                };
        }
    }
}
