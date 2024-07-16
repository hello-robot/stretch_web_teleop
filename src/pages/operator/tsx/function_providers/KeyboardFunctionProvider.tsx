import React from "react";
import { buttonFunctionProvider } from "operator/tsx/index";
import { FunctionProvider } from "./FunctionProvider";
import { ButtonPadButton, ButtonFunctions } from "./ButtonFunctionProvider";
import { ActionMode } from "../utils/component_definitions";

export enum Mode {
  Base = "base",
  Arm = "arm",
  Wrist = "wrist",
}

export class KeyboardFunctionProvider extends FunctionProvider {
  constructor() {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
  }

  private getButtonPadButton(
    KeyboardFunction: Mode,
    key: string,
  ): ButtonPadButton {
    // TODO
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
            keyInput = ButtonPadButton.WristRollLeft;
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
            keyInput = ButtonPadButton.WristRollRight;
            break;
          default:
            break;
        }
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
    let button = this.getButtonPadButton(keyboardFunction, key);
    return buttonFunctionProvider.provideFunctions(button);
  }
}

// export default function KeyboardFunctionProvider() {
//   const [mode, setMode] = React.useState<Mode>();

//   // constructor(){
//   //   super();
//   //   this.provideFunctions = this.provideFunctions.bind(this);
//   // }

//   // public provideFunctions()
//   const handleKeyPress = React.useCallback(
//     (event) => {
//       switch (event.key) {
//         case "w":
//           HandleW();
//           break;
//         case "a":
//           HandleA();
//           break;
//         case "s":
//           HandleS();
//           break;
//         case "d":
//           HandleD();
//           break;
//         case "1":
//           setMode(Mode.Base);
//           console.log("base mode enabled");
//           break;
//         case "2":
//           setMode(Mode.Arm);
//           console.log("arm mode enabled");
//           break;
//         case "3":
//           setMode(Mode.Wrist);
//           console.log("wrist mode enabled");
//           break;
//         default:
//           break;
//       }

//       switch (event.keyCode) {
//         case 38:
//           console.log("camera moved up");
//           break;
//         case 37:
//           console.log("camera moved left");
//           break;
//         case 40:
//           console.log("camera moved down");
//           break;
//         case 39:
//           console.log("camera moved right");
//           break;
//       }
//     },
//     [mode],
//   );

//   const HandleW = () => {
//     switch (mode) {
//       case Mode.Base:
//         console.log("w pressed; base mode");
//         break;
//       case Mode.Arm:
//         console.log("w pressed; arm mode");
//         break;
//       case Mode.Wrist:
//         console.log("w pressed; wrist mode");
//         break;
//       default:
//         break;
//     }
//   };

//   const HandleA = () => {
//     switch (mode) {
//       case Mode.Base:
//         console.log("a pressed; base mode");
//         break;
//       case Mode.Arm:
//         console.log("a pressed; arm mode");
//         break;
//       case Mode.Wrist:
//         console.log("a pressed; wrist mode");
//         break;
//       default:
//         break;
//     }
//   };

//   const HandleS = () => {
//     switch (mode) {
//       case Mode.Base:
//         console.log("s pressed; base mode");
//         break;
//       case Mode.Arm:
//         console.log("s pressed; arm mode");
//         break;
//       case Mode.Wrist:
//         console.log("s pressed; wrist mode");
//         break;
//       default:
//         break;
//     }
//   };

//   const HandleD = () => {
//     switch (mode) {
//       case Mode.Base:
//         console.log("d pressed; base mode");
//         break;
//       case Mode.Arm:
//         console.log("d pressed; arm mode");
//         break;
//       case Mode.Wrist:
//         console.log("d pressed; wrist mode");
//         break;
//       default:
//         break;
//     }
//   };

//   React.useEffect(() => {
//     window.addEventListener("keydown", handleKeyPress);
//     return () => {
//       window.removeEventListener("keydown", handleKeyPress);
//     };
//   }, [handleKeyPress]);

//   return (
//     <>
//       <div>
//         <button>{mode}</button>
//       </div>
//     </>
//   );
// }
