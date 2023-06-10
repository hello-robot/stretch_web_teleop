# Operator page TypeScript files

## Directory outline

A list of directories and their contents:
* `basic_components/`
    * simple components used in multiple places throughout the code (such as specific buttons or popups)
* `default_layouts/`
    * Javascript object definitions for the different default layouts avaliable for the user to load
    * The default layouts provide a safe state of the interface that can always be reloaded
* `function_providers/`
    * subclasses of the `FunctionProvider` class which are used during render to determine the functionality of controls (such as button pads or predictive display)
* `layout_components/` 
    * components which can be dynamically added, changed, or removed by the user
    * includes things like camera views, button pads, panels, and tabs
* `static_components/`
    * components which are always visible, such as the action mode button, sidebar, speed control, or customization button
* `utils/`
    * typescript files which contain logic seperated from the React components
* `Operator.tsx/`
    * Highest level React component for the entire operator page
* `index.tsx`
    * Logic for connecting with the robot browser using WebRTC
    * Initializes state for the application
    * Renders `Operator`

# Creating Your Own Components

To create a new component you'll want to follow these steps:

1. **Create a new type** for your component in `ComponentType` in `utils/component_definitions.tsx`.
    * If there are going to be subtypes of your component then define an id for each of the subtypes like `CameraViewId` in `utils/component_definitions.tsx`
    * If you component needs any other field in order for it to render (such as a `TabDefinition` having a `label`), then create a seperate definition for your component with those fields.

1. **Create a new file** in `layout_components` with the React code for your new component. The React functional component should take `CustomizableComponentProps` as its props. A field in `CustomizableComponentProps` is the `ComponentDefinition`, so you should be able to access all of the fields in the components defintion there. Here are some more details about the React component you create:

    1. **Selecting the component**. In your React component code, make sure there's a way to select the component if the user should be able to move it around the layout. This means calling `props.sharedState.onSelect` with the definition and path of the component. For an example, see the `onSelect` function defined in the `ButtonPad` functional component in `layout_components/ButtonPad.tsx`
    1. **Setting the class name**. The standard is to use the [`className()`](../../../shared/util.tsx) util function to set the "customizing" and "selected" flags in a component classname. See [`CameraView.tsx`](./layout_components/CameraView.tsx) for an example.

1. **Add the component to `CustomizableComponent`**. In the switch statement within `CustomizableComponent` add a case associating the `ComponentType` for your component with the React functional component.

1. **Create a definition** for your component in [`componentDescription()`](./static_components/Sidebar.tsx). Add a case to the switch statement for your `ComponentType`. This function tells the sidebar what to display (rather than "Selected: none") when your new component is selected.

> TODO: I believe these are all the steps to create a new component. If any steps are missing or the process changes, make sure to update the instructions here.