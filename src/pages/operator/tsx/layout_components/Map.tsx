import React from "react";
import Nav2d from "react-nav2djs";
import { CustomizableComponentProps } from "./CustomizableComponent";
import { MapDefinition } from "../utils/component_definitions";
import "operator/css/Map.css"

export const Map = (props: CustomizableComponentProps) => {
    const definition = props.definition as MapDefinition

    // Reference to the video element
    const mapRef = React.useRef<HTMLDivElement>();

    /** Callback when Map is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<SVGSVGElement>) => {
        // Make sure the container of the button pad doesn't get selected
        // event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }

    console.log("canvas ", document.getElementById("mapCanvas"))

     // Constrain the width or height when the stream gets too large
     React.useEffect(() => {
        const resizeObserver = new ResizeObserver(entries => {

            // height and width of area around the video stream
            const { height, width } = entries[0].contentRect;

            console.log("canvas ", document.getElementById("mapCanvas"))
            // height and width of video stream
            // if (!videoRef?.current) return;
            // const videoRect = videoRef.current.getBoundingClientRect();

            // if (videoRect.height > height) {
            //     setConstrainedHeight(true);
            // } else if (videoRect.width > width) {
            //     setConstrainedHeight(false);
            // }
        });
        // if (!videoAreaRef?.current) return;
        // resizeObserver.observe(videoAreaRef.current);
        // return () => resizeObserver.disconnect();
    }, []);

    return (
        <div className="map-container">
            <Nav2d 
                ros={definition.ros}
                id={"map"}
                width={300}
                height={300}
                serverName="/move_base"
            />
        </div>
    )
}

// var ratio = Math.min(493 / this.width, 1328 / this.height);
//     canvas.width = this.width * ratio;
//     canvas.height = this.height * ratio;