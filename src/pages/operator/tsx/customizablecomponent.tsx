import { ComponentDef } from "./componentdefinitions";
import { FunctionProvider } from "./functionprovider";

/** Properties for any of the customizable components: tabs, video streams, or
 * button pads.
 */
export interface CustomizableComponentProps {
    definition: ComponentDef;
    customizing: boolean;
    path: string;
    functionProvider: FunctionProvider;
}