import React from "react";
import { buttonFunctionProvider } from "operator/tsx/index";
import { ButtonPadButton, ButtonFunctions } from "./ButtonFunctionProvider";
import { FunctionProvider } from "./FunctionProvider";
import { AdaptiveControl } from "./AdaptiveControl";

export class AdaptiveFunctionProvider extends FunctionProvider {
  constructor() {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
  }

  private getButtonPadButton(): ButtonPadButton {
    // let buttonInput: ButtonPadButton;
    // if(aPressed){
    //     buttonInput = ButtonPadButton.BaseForward;
    // } else { buttonInput = null; }
    // return buttonInput;
  }

  public provideFunctions(): ButtonFunctions | null {
    const button = this.getButtonPadButton();
    console.log(
      "**********************",
      `${button}`,
      "**********************",
    );
    return buttonFunctionProvider.provideFunctions(button);
  }
}
