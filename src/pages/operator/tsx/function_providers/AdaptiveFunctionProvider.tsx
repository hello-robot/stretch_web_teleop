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
      case 12:
        console.log("Up dpad pressed");
        keyInput = ButtonPadButton.CameraTiltUp;
        break;
      case 13:
        keyInput = ButtonPadButton.CameraTiltDown;
        break;
      case 14:
        keyInput = ButtonPadButton.CameraPanLeft;
        break;
      case 15:
        keyInput = ButtonPadButton.CameraPanRight;
        break;
      default:
        console.log("Button ", buttonIndex, " pressed");
    }
    console.log(keyInput);
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
