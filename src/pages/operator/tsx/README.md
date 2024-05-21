# Operator page TypeScript files

For more info on [how to create your own component](./create_component.md).

For more info on [how the customization logic works](./customization_logic.md).

## Directory outline

A list of directories and their contents:

- `basic_components/`
  - simple components used in multiple places throughout the code (such as specific buttons or popups)
- `default_layouts/`
  - Javascript object definitions for the different default layouts available for the user to load
  - The default layouts provide a safe state of the interface that can always be reloaded
- `function_providers/`
  - subclasses of the `FunctionProvider` class which are used during render to determine the functionality of controls (such as button pads or predictive display)
- `layout_components/`
  - components which can be dynamically added, changed, or removed by the user
  - includes things like camera views, button pads, panels, and tabs
- `static_components/`
  - components which are always visible, such as the action mode button, sidebar, speed control, or customization button
- `utils/`
  - typescript files which contain logic separated from the React components
- `Operator.tsx/`
  - Highest level React component for the entire operator page
- `index.tsx`
  - Logic for connecting with the robot browser using WebRTC
  - Initializes state for the application
  - Renders `Operator`

# Render Logic Flow

This diagram shows the flow of logic between classes and components while the Operator Page renders.

![Operator render logic flow](../../../../documentation/assets/operator/render_logic_flow.png)

**`StorageHandler`**
: When the `Operator` first renders, it gets the `layout` from the `StorageHandler` (to preserve state between page reloads). Whenever the user changes the `layout`, `Operator` will save the updated state with `StorageHandler`.

**`Operator`**
: creates a `sharedState` object with relevant information for all components in the layout, then passes the `layout` and `sharedState` to the `LayoutArea`.

**`LayoutArea`**
: corresponds to "Layout" in the Component Hierarchy. This renders the individual components in the layout, with `DropZone`'s in between so that components can be moved in customize mode.

**`CustomizableComponent`**
: a single component in the layout, the customizable component renders a different subcomponent based on the `type` in the `ComponentDefinition`.

**`FunctionProvider`**
: takes the action mode and speed control from `ActionMode` and `SpeedControl` respectively. Returns a set of functions for how different controls should behave, for example `onClick` and `onRelease` for a button on a `ButtonPad`.

**`ButtonPad` and other controls**
: when `ButtonPad` or another control renders, it gets the set of functions from the `FunctionProvider`.
