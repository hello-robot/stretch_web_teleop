import React from "react";
import Nav2d from "react-nav2djs";
import 'latest-createjs';
import { CustomizableComponentProps, isSelected } from "./CustomizableComponent";
import { MapDefinition } from "../utils/component_definitions";
import { mapFunctionProvider, occupancyGrid } from "operator/tsx/index";
import "operator/css/Map.css"
import { ROSViewer } from "../static_components/ROSViewer";
import { ROSOccupancyGridClient } from "../static_components/ROSOccupancyGridClient";
import { AMCLPose, ROSOccupancyGrid, ROSPose, className } from "shared/util";
import ROSLIB from "roslib";

export enum MapFunction {
    GetMap,
    GetPose,
    MoveBase
}

export const Map = (props: CustomizableComponentProps) => {
    const definition = props.definition as MapDefinition
    const [active, setActive] = React.useState<boolean>(false);
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    // Reference to the video element
    const mapRef = React.useRef<HTMLDivElement>();

    /** Callback when Map is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<SVGSVGElement>) => {
        // Make sure the container of the button pad doesn't get selected
        // event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }


     // Constrain the width or height when the stream gets too large
     React.useEffect(() => {
        var viewer = new ROSViewer({
            divID: 'map',
            width: 257,
            height: 385
        });
        var occupancyGrid = new ROSOccupancyGridClient({
            map: getMap,
            moveBaseFn: moveBase,
            getPoseFn: getPose,
            rootObject: viewer.scene
        })
        // // Scale the canvas to fit to the map
        viewer.scaleToDimensions(occupancyGrid.width, occupancyGrid.height);
        // viewer.shift(0, -occupancyGrid.height);
    }, []);

    function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }

    let getMap = mapFunctionProvider.provideFunctions(MapFunction.GetMap) as ROSOccupancyGrid
    let getPose = mapFunctionProvider.provideFunctions(MapFunction.GetPose) as () => ROSLIB.Transform
    let moveBase = mapFunctionProvider.provideFunctions(MapFunction.MoveBase) as (pose: ROSPose) => void

    return (
        <div className="map-container">
            {/* <Nav2d 
                map={getMap()}
                id={"map"}
                width={300}
                height={300}
                serverName="/move_base"
            /> */}
            <div id="map" className={className("map", { customizing, selected, active })} onClick={handleSelect}></div>
        </div>
    )
}

// var ratio = Math.min(493 / this.width, 1328 / this.height);
//     canvas.width = this.width * ratio;
//     canvas.height = this.height * ratio;