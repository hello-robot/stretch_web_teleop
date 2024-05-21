import React from "react";
import {
  ComponentDefinition,
  ComponentType,
} from "../utils/component_definitions";
import { DropZoneState } from "./DropZone";
import { Panel } from "./Panel";
import { RemoteStream } from "shared/util";
import { ButtonPad } from "./ButtonPad";
import { CameraView } from "./CameraView";
import { PredictiveDisplay } from "./PredictiveDisplay";
import { ButtonStateMap } from "../function_providers/ButtonFunctionProvider";
import { ButtonGrid } from "./ButtonGrid";
import { VirtualJoystick } from "./VirtualJoystick";
import { Map } from "./Map";
import { RunStopButton } from "../static_components/RunStop";
import { BatteryGuage } from "../static_components/BatteryGauge";

/** State required for all elements */
export type SharedState = {
  customizing: boolean;
  /** Called when user clicks on a component */
  onSelect: (def: ComponentDefinition, path?: string) => void;
  /** Remote robot video streams */
  remoteStreams: Map<string, RemoteStream>;
  /** State required for all dropzones */
  dropZoneState: DropZoneState;
  /** Path to the active component */
  selectedPath?: string;
  /** Mapping of each button pad function to a {@link ButtonState} */
  buttonStateMap?: ButtonStateMap;
  /** Whether or not to hide the button labels */
  hideLabels?: boolean;
  /** Whether or not the beta teleop cameras are being used */
  hasBetaTeleopKit: boolean;
};

/** Properties for any of the customizable components: tabs, video streams, or
 * button pads.
 */
export type CustomizableComponentProps = {
  /**
   * Path to the component
   * @example "0-2" would represent the 2nd child of the 0th element in the layout
   */
  path: string;
  /**
   * Definition of the component (all the info required to know that type
   * of component to render
   */
  definition: ComponentDefinition;
  /** see {@link SharedState} */
  sharedState: SharedState;
};

/**
 * Takes a definition for a component and returns the react component.
 *
 * @note switch on the component definition's `type` field
 * @returns rendered component
 */
export const CustomizableComponent = (props: CustomizableComponentProps) => {
  if (!props.definition.type) {
    throw new Error(`Component at ${props.path} is missing type`);
  }

  // switch on the component type to render specific type of component
  switch (props.definition.type) {
    case ComponentType.Panel:
      return <Panel {...props} />;
    case ComponentType.ButtonPad:
      return <ButtonPad {...props} />;
    case ComponentType.CameraView:
      return <CameraView {...props} />;
    case ComponentType.PredictiveDisplay:
      return <PredictiveDisplay {...props} />;
    case ComponentType.ButtonGrid:
      return <ButtonGrid {...props} />;
    case ComponentType.VirtualJoystick:
      return <VirtualJoystick {...props} />;
    case ComponentType.Map:
      return <Map {...props} />;
    case ComponentType.RunStopButton:
      return <RunStopButton {...props} />;
    case ComponentType.BatteryGuage:
      return <BatteryGuage {...props} />;
    default:
      throw Error(
        `CustomizableComponent cannot render component of unknown type: ${props.definition.type}\nYou may need to add a case for this component in the switch statement in CustomizableComponent.`,
      );
  }
};

/**
 * Checks if the component is currently selected
 * @returns true if selected, otherwise false
 */
export function isSelected(props: CustomizableComponentProps): boolean {
  return props.path === props.sharedState.selectedPath;
}
