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
    * components which are always visible, such as the action mode button, sidebar, velocity control, or customization button
* `utils/`
    * typescript files which contain logic seperated from the React components
