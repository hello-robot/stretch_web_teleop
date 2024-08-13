import React from "react";
import { buttonFunctionProvider } from "operator/tsx/index";
import { ButtonPadButton, ButtonFunctions } from "./ButtonFunctionProvider";
import { FunctionProvider } from "./FunctionProvider";
import e from "express";

export class AdaptiveFunctionProvider extends FunctionProvider {
  constructor() {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
  }

  private getButtonPadButton(buttonIndex: number) {
    let keyInput: ButtonPadButton | null = null;
    switch (buttonIndex) {
      case 0:
        console.log("A pressed");
        keyInput = ButtonPadButton.BaseForward;
        break;
      case 1:
        console.log("B pressed");
        keyInput = ButtonPadButton.BaseReverse;
        break;
      case 2:
        console.log("X pressed");
        keyInput = ButtonPadButton.BaseRotateLeft;
      case 3:
        console.log("Y pressed");
        keyInput = ButtonPadButton.BaseRotateRight;
        break;
      case 4:
        console.log("LB pressed");
        // keyInput =
        break;
      case 5:
        console.log("RB pressed");
        // keyInput =
        break;
      case 6:
        console.log("LT pressed");
        // keyInput =
        break;
      case 7:
        console.log("RT pressed");
        // keyInput =
        break;
      case 8:
        console.log("Select pressed"); //Button with the two boxes
        // keyInput =
        break;
      case 9:
        console.log("Start pressed"); //Button with three lines
        // keyInput =
        break;
      case 10:
        console.log("L3 pressed");
        // keyInput = ;
        break;
      case 11:
        console.log("R3 pressed");
        // keyInput =
        break;
      case 12:
        console.log("Up dpad pressed");
        keyInput = ButtonPadButton.CameraTiltUp;
        break;
      case 13:
        console.log("Down dpad pressed");
        keyInput = ButtonPadButton.CameraTiltDown;
        break;
      case 14:
        console.log("Left dpad pressed");
        keyInput = ButtonPadButton.CameraPanLeft;
        break;
      case 15:
        console.log("Right dpad pressed");
        keyInput = ButtonPadButton.CameraPanRight;
        break;
      case 16:
        console.log("XBOX pressed");
        // keyInput =
        break;
      default:
        console.log("Button ", buttonIndex, " pressed");
    }
    return keyInput;
  }

  public provideFunctions(index: number): ButtonFunctions | null {
    const button = this.getButtonPadButton(index);
    if (button !== null) {
      return buttonFunctionProvider.provideFunctions(button);
    } else {
      return null;
    }
  }
}
