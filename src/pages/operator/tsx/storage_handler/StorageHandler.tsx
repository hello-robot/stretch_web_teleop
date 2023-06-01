import { MAIN_BRANCH_LAYOUT } from "../default_layouts/main_branch";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/study_branch";
import { LayoutDefinition } from "./componentdefinitions";

export type DefaultLayoutName = "Study branch" | "Main branch";
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } = {
    "Study branch": STUDY_BRANCH_LAYOUT,
    "Main branch": MAIN_BRANCH_LAYOUT
}

export abstract class StorageHandler {
    constructor(props: {onStorageHandlerReadyCallback: () => void}) {
        this.onReadyCallback = props.onStorageHandlerReadyCallback.bind(this)
    }

    public abstract loadCustomLayout(layoutName: string): LayoutDefinition;

    public abstract saveCustomLayout(layout: LayoutDefinition, layoutName: string): void;

    public abstract saveCurrentLayout(layout: LayoutDefinition): void;

    public abstract loadCurrentLayout(): LayoutDefinition | null;

    public onReadyCallback: () => void;

    public abstract getCustomLayoutNames(): string[];

    public loadCurrentLayoutOrDefault(): LayoutDefinition {
        const currentLayout = this.loadCurrentLayout();
        if (!currentLayout) return Object.values(DEFAULT_LAYOUTS)[0];
        console.log('loading saved layout')
        return currentLayout;
    }

    public getDefaultLayoutNames(): string[] {
        return Object.keys(DEFAULT_LAYOUTS);
    }

    public loadDefaultLayout(layoutName: DefaultLayoutName): LayoutDefinition {
        return DEFAULT_LAYOUTS[layoutName];
    }
}