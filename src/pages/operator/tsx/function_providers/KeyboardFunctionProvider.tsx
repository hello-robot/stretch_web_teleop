import React from "react";
import { buttonFunctionProvider } from "operator/tsx/index";
import { FunctionProvider } from "./FunctionProvider";
import { ButtonPadButton, ButtonFunctions } from "./ButtonFunctionProvider";
import { ActionMode } from "../utils/component_definitions";
// import { keyState } from "../layout_components/KeyboardControl"

export enum Mode {
  Base = "Base",
  Arm = "Arm",
  Wrist = "Wrist",
}

// const [keyState, setKeyState] = React.useState({
//   W: false,
//   A: false,
//   S: false,
//   D: false,
//   J: false,
//   K: false,
//   UpArrow: false,
//   LeftArrow: false,
//   DownArrow: false,
//   RightArrow: false,
// });

export class KeyboardFunctionProvider extends FunctionProvider {
  constructor() {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
    /// buttonFunctionProvider.updateJointStates = buttonFunctionProvider.updateJointStates.bind(this);
  }

  private getButtonPadButton(
    KeyboardFunction: Mode,
    key: string,
  ): ButtonPadButton {
    let keyInput: ButtonPadButton;
    switch (key) {
      case "w":
        switch (KeyboardFunction) {
          case Mode.Base:
            console.log("w pressed; base mode");
            keyInput = ButtonPadButton.BaseForward;
            break;
          case Mode.Arm:
            console.log("w pressed; arm mode");
            keyInput = ButtonPadButton.ArmLift;
            break;
          case Mode.Wrist:
            console.log("w pressed; wrist mode");
            keyInput = ButtonPadButton.WristPitchUp;
            break;
          default:
            break;
        }
        break;
      case "a":
        switch (KeyboardFunction) {
          case Mode.Base:
            console.log("a pressed; base mode");
            keyInput = ButtonPadButton.BaseRotateLeft;
            break;
          case Mode.Arm:
            console.log("a pressed; arm mode");
            keyInput = ButtonPadButton.ArmRetract;
            break;
          case Mode.Wrist:
            console.log("a pressed; wrist mode");
            keyInput = ButtonPadButton.WristRotateIn;
            break;
          default:
            break;
        }
        break;
      case "s":
        switch (KeyboardFunction) {
          case Mode.Base:
            console.log("s pressed; base mode");
            keyInput = ButtonPadButton.BaseReverse;
            break;
          case Mode.Arm:
            console.log("s pressed; arm mode");
            keyInput = ButtonPadButton.ArmLower;
            break;
          case Mode.Wrist:
            console.log("s pressed; wrist mode");
            keyInput = ButtonPadButton.WristPitchDown;
            break;
          default:
            break;
        }
        break;
      case "d":
        switch (KeyboardFunction) {
          case Mode.Base:
            console.log("d pressed; base mode");
            keyInput = ButtonPadButton.BaseRotateRight;
            break;
          case Mode.Arm:
            console.log("d pressed; arm mode");
            keyInput = ButtonPadButton.ArmExtend;
            break;
          case Mode.Wrist:
            console.log("d pressed; wrist mode");
            keyInput = ButtonPadButton.WristRotateOut;
            break;
          default:
            break;
        }
        break;
      case "q":
        switch (KeyboardFunction) {
          case Mode.Wrist:
            console.log("q pressed; wrist mode");
            keyInput = ButtonPadButton.WristRollLeft;
            break;
          default:
            break;
        }
        break;
      case "e":
        switch (KeyboardFunction) {
          case Mode.Wrist:
            console.log("e pressed; wrist mode");
            keyInput = ButtonPadButton.WristRollRight;
            break;
          default:
            break;
        }
        break;
      case "j":
        console.log("j pressed; gripper closed");
        keyInput = ButtonPadButton.GripperClose;
        break;
      case "k":
        console.log("k pressed; gripper opened");
        keyInput = ButtonPadButton.GripperOpen;
        break;
      case "ArrowUp":
        console.log("camera moved up");
        keyInput = ButtonPadButton.CameraTiltUp;
        break;
      case "ArrowLeft":
        console.log("camera moved left");
        keyInput = ButtonPadButton.CameraPanLeft;
        break;
      case "ArrowDown":
        console.log("camera moved down");
        keyInput = ButtonPadButton.CameraTiltDown;
        break;
      case "ArrowRight":
        console.log("camera moved right");
        keyInput = ButtonPadButton.CameraPanRight;
        break;
      case "":
        console.log("Key Released");
      default:
        console.log("Unmapped key", key);
        break;
    }
    return keyInput;
  }

  public provideFunctions(
    keyboardFunction: Mode,
    key: string,
  ): ButtonFunctions {
    // switch()
    let button = this.getButtonPadButton(keyboardFunction, key);
    return buttonFunctionProvider.provideFunctions(button);
  }
}