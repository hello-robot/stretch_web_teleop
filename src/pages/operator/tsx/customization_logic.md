
# **Overview of Customization Logic**
# Layout

The `layout` is passed into `Operator` as a property. The `layout` defines which components are in the layout and how they're arranged. Below is a very simple example:

```ts
import { LayoutDefinition, ComponentType, ActionMode, CameraViewId, ButtonPadId, OverheadVideoStreamDef, TabDefinition, PanelDefinition } from "/utils/component_definitions";

export const SIMPLE_LAYOUT: LayoutDefinition = {
    // All components have a type
    type: ComponentType.Layout,
    // If voice control should be displayed on the operator page
    displayVoiceControl: false,
    // The state of the action mode dropdown
    actionMode: ActionMode.StepActions,

    // The customizable components in the layout
    children: [
        {
            // The layout contains a single panel
            type: ComponentType.Panel,
            children: [
                {
                    // The panel contains a single tab
                    type: ComponentType.SingleTab,
                    // The title of the tab is "Tab One"
                    label: 'Tab One',
                    children: [
                        {
                            // Tab contains a single camera view
                            type: ComponentType.CameraView,
                            // From the Stretch overhead camera
                            id: CameraViewId.overhead,
                            children: [
                                {
                                    // Camera view has a button pad overlay
                                    type: ComponentType.ButtonPad,
                                    // Button pad to control the base 
                                    id: ButtonPadId.Drive
                                }
                            ]
                        } as OverheadVideoStreamDef
                    ]
                } as TabDefinition
            ]
        } as PanelDefinition
    ]
}
```

The layout is an object containing a set of nested `ComponentDefinitions`, where each `ComponentDefinition` represents the details for a `CustomizableComponent` to render.

For more layout examples, see the predefined default layouts can be found in `./tsx/default_layouts/`.

All of the definitions for the different component definitions can be found in `./tsx/utils/component_definitions.tsx`

# Logic for Adding, Rearranging, and Removing Components


![drop zone example](../../../../documentation/assets/operator/dropzones.png)

## Selecting a Component

`CustomizableComponents` can be **selected** in the `LayoutArea` or from the `Sidebar`'s component provider. The screenshot above shows the Realsense Camera View selected from the `Sidebar`'s component provider.

`Operator` **keeps track of the selected component** using its `selectedPath` and `selectedDefinition` fields. 
* the `selectedDefinition` is the `ComponentDefinition` for the selected object
* the `selectedPath` is a string representing the path to the component in the layout. 
* when nothing is selected, these fields are `undefined`

> For example: the path to the right video stream would be `0-0-1`, because it's at the 1 index in the tab, which is at the 0 index in the panel, which is in the 0 index in the layout.

## DropZones

The `LayoutArea` is made up of two main elements: `CustomizableComponent` and `DropZone`. 

`DropZone` shows the user where they can place elements. In the image above the **gray areas with dotted outlines** are each a `DropZone`, where the user can click to place the camera view.

During customization mode, a `DropZone` will only appear if the `canDrop()` function (defined in `DropZone.tsx`) evaluates to `true`.

When a user clicks on a `DropZone`, it executes a callback to `handleDrop()` (defined in `Operator.tsx`) with the path to the `DropZone` in the `LayoutArea`. This then modifies the `layout` structure stored in `Operator` to represent the change.

## Adding a Component

* When a component is selected from the `Sidebar`, `selectedDefinition` is set with the corresponding `type` (and `id` if applicable). 
* `selectedPath` is undefined since components in the sidebar are not in the layout and thus do not have a `path`.
* When the user clicks on a `DropZone` the new `ComponentDefinition` is added to the `layout` in `Operator`

## Moving a Component

* When a component is selected from `LayoutArea`, the `CustomizableComponent` calls `handleSelect()` in `Operator` with its `path` and `definition`. This sets the `selectedPath` and `selectedDefinition` state in `Operator`.
* When the user clicks on a `DropZone` the component is removed from its old position in the `layout` and added to a new position at the `path` of the clicked `DropZone`.

## Removing a Component
* Once a component from the `LayoutArea` is selected, the delete button in the bottom of the sidebar will become enabled. 
* When the user clicks on the delete button, it calls `handleDelete()` in `Operator` which removes the component from the layout.

