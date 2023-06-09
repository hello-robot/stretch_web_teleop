# Operator page

The operator page is what users see while teleoperating the robot.

For more info on the directory file structure see [documentation here](./tsx/README.md).

## Operator Page

![Operator Page](../../../documentation/assets/operator_page.png)

## Component Hiearchy

Below is a diagram showing the hiearchy between components in the operator page.

![operator page component hiearcy](../../../documentation/assets/component_hiearchy.png)

* Operator
: contains all subcomponents

* Header
: fixed at the top of the screen, this displays controls that the user always has access to
    * Action Mode
    : dropdown to switch between step-actions, press-release, and click-click action modes
    * Speed Control
    : Set of buttons labeled from "slowest" to "fastest" which control the scaling of the robots speed for all joints and all controls
    * Customize button
    : Button on the right side of the header which enters/leaves customization mode