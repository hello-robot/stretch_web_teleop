import { FunctionProvider } from "./FunctionProvider";
import { TextToSpeechFunction } from "../layout_components/TextToSpeech";
import { StorageHandler } from "../storage_handler/StorageHandler";

export class TextToSpeechFunctionProvider extends FunctionProvider {
  private storageHandler: StorageHandler;

  constructor(storageHandler: StorageHandler) {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
    this.storageHandler = storageHandler;
  }

  public provideFunctions(textToSpeechFunction: TextToSpeechFunction) {
    switch (textToSpeechFunction) {
      case TextToSpeechFunction.Play:
        return (text: string) => {
          console.log("Playing text: ", text);
        };
      case TextToSpeechFunction.Stop:
        return () => {
          console.log("Stopping text");
        };
      case TextToSpeechFunction.SaveText:
        return (text: string) => {
          console.log("Saving text: ", text);
        };
      case TextToSpeechFunction.DeleteText:
        return (textID: number) => {
          console.log("Deleting text with ID: ", textID);
        };
      case TextToSpeechFunction.SavedTexts:
        return () => {
          console.log("Getting saved texts");
          return ["Hello", "Goodbye"];
        };
    }
  }
}
