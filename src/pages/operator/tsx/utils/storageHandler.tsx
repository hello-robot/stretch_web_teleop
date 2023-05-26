import { MAIN_BRANCH_LAYOUT } from "../default_layouts/main_branch";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/study_branch";
import { LayoutDefinition } from "./componentdefinitions";

export type DefaultLayoutName = "Study branch" | "Main branch";
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } = {
    "Study branch": STUDY_BRANCH_LAYOUT,
    "Main branch": MAIN_BRANCH_LAYOUT
}

const USER_LAYOUT_SAVE_KEY = "user_custom_layout";

export abstract class StorageHandler {

    public abstract loadLayout(layoutName?: string): LayoutDefinition;

    public abstract saveLayout(layout: LayoutDefinition, layoutName?: string): void;
}

export class LocalStorageHandler extends StorageHandler {
    public loadLayout(layoutName?: string): LayoutDefinition {
        console.log('loading layout' + layoutName)
        if (!layoutName) {
            const storedJson = localStorage.getItem(USER_LAYOUT_SAVE_KEY);
            if (storedJson) {
                console.log('loaded stored layout');
                console.log(storedJson);
                return JSON.parse(storedJson);
            }
            return STUDY_BRANCH_LAYOUT;
        }
        if (!(layoutName in DEFAULT_LAYOUTS))
            throw Error(`could not find layout name ${DEFAULT_LAYOUTS}`);
        console.log('returning default layout')
        return DEFAULT_LAYOUTS[layoutName as DefaultLayoutName]!;
    }

    public saveLayout(layout: LayoutDefinition, layoutName?: string): void {
        localStorage.setItem(layoutName || USER_LAYOUT_SAVE_KEY, JSON.stringify(layout));
    }
}