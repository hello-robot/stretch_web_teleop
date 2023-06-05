import React from "react";
import { className, gripperProps, navigationProps, realsenseProps, RemoteStream } from "shared/util";
import { VideoStreamDef, ComponentType, VideoStreamId, ComponentDefinition, OverheadVideoStreamDef, RealsenseVideoStreamDef } from "../utils/componentdefinitions";
import { ButtonPad } from "./buttonpads";
import { CustomizableComponentProps, SharedState } from "./customizablecomponent";
import { DropZone } from "./dropzone";
import { PredictiveDisplay } from "./predictivedisplay";
import "operator/css/videostreamcomponent.css"
import { buttonFunctionProvider, underVideoFunctionProvider } from "..";
import { ButtonPadButton, panTiltButtons } from "../functionprovider/buttonpads";
import { OverheadButtons, overheadButtons, realsenseButtons, RealsenseButtons, UnderVideoButton, UnderVideoFunctionProvider } from "../functionprovider/undervideobuttons";
import { CheckToggleButton } from "../basic_components/CheckToggleButton";

/**
 * Displays a video stream with an optional button pad overlay
 * 
 * @param props properties
 */
export const VideoStreamComponent = (props: CustomizableComponentProps) => {
    // Reference to the video element
    const videoRef = React.useRef<HTMLVideoElement>(null);
    // X and Y position of the cursor when user clicks on the video
    const [clickXY, setClickXY] = React.useState<[number, number] | null>(null);
    const definition = props.definition as VideoStreamDef;
    if (!definition.children) console.warn(`Video stream definition at ${props.path} should have a 'children' property.`);
    // Get the stream to display inside the video
    const stream: MediaStream = getStream(definition.id, props.sharedState.remoteStreams);
    // Refrence to the div immediately around the video element
    const videoAreaRef = React.useRef<HTMLDivElement>(null);
    // Boolean representing if the video stream needs to be constrained by height
    // (constrained by width otherwise)
    const [constrainedHeight, setConstrainedHeight] = React.useState<boolean>(false);

    executeVideoSettings(definition);

    // Create the overlay
    const overlayDefinition = (definition.children && definition.children.length > 0) ? definition.children[0] : undefined;
    const videoAspectRatio = getVideoAspectRatio(definition);
    const overlay = createOverlay(overlayDefinition, props.path, props.sharedState, videoAspectRatio);

    // Update the source of the video stream 
    React.useEffect(() => {
        if (!videoRef?.current) return;
        videoRef.current.srcObject = stream;
    }, [stream]);

    const { customizing } = props.sharedState;
    const selected = props.path === props.sharedState.activePath;
    const videoClass = className("video-canvas", { customizing, selected })

    /** Mark this video stream as selected */
    function selectSelf() {
        props.sharedState.onSelect(props.definition, props.path);
        setClickXY(null);
    }

    /** Mark the button pad child as selected */
    function selectChild() {
        props.sharedState.onSelect(overlayDefinition!, props.path + '-0');
        setClickXY(null);
    }

    /** Opens a popup  */
    function handleClick(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();

        // If no button pad overlay then select self and return
        if (!overlayDefinition || overlayDefinition.type !== ComponentType.ButtonPad) {
            selectSelf();
            return;
        }

        // Create context menu popup where user can choose between selecting 
        // the button pad or the video stream
        const { clientX, clientY } = event;
        const { left, top } = videoRef.current!.getBoundingClientRect();
        const x = clientX - left;
        const y = clientY - top;
        setClickXY([x, y]);
    }

    // Constrain the width or height when the stream gets too large
    React.useEffect(() => {
        console.log('use effect')
        const resizeObserver = new ResizeObserver(entries => {

            // height and width of area around the video stream
            const { height, width } = entries[0].contentRect;

            // height and width of video stream
            if (!videoRef?.current) return;
            const videoRect = videoRef.current.getBoundingClientRect();

            if (videoRect.height > height) {
                setConstrainedHeight(true);
            } else if (videoRect.width > width) {
                setConstrainedHeight(false);
            }
        });
        if (!videoAreaRef?.current) return;
        resizeObserver.observe(videoAreaRef.current);
        return () => resizeObserver.disconnect();
    }, []);


    const overlayContainer = (
        <div
            className={className("video-overlay-container", { customizing, selected })}
            // style={overlayDimensions}
            onClick={customizing ? handleClick : undefined}
        >
            {
                // Display overlay on top of video stream
                overlay ? overlay :
                    <DropZone
                        path={props.path + "-0"}
                        sharedState={props.sharedState}
                        parentDef={props.definition}
                    />
            }
            {
                // When clickXY is set, display context menu
                clickXY ? <SelectContexMenu
                    clickXY={clickXY}
                    selectSelf={selectSelf}
                    selectChild={selectChild}
                    clickOut={() => setClickXY(null)}
                /> : undefined
            }
        </div>
    )
    // If the video is from the Realsense camera then include the pan-tilt 
    // buttons around the video, otherwise return the video
    const videoComponent = (props.definition.id === VideoStreamId.realsense) ?
        (
            <div className={className("realsense-pan-tilt-grid", { constrainedHeight })}>
                {panTiltButtons.map(dir => <PanTiltButton direction={dir} key={dir} />)}
                <div className="video-area" style={{ gridRow: 2, gridColumn: 2 }} ref={videoAreaRef}>
                    <video
                        ref={videoRef}
                        autoPlay
                        muted={true}
                        className={className(videoClass, { constrainedHeight })}
                    />
                    {overlayContainer}
                </div>
            </div>
        )
        :
        (
            <div className="video-area" style={{ gridRow: 2, gridColumn: 1 }} ref={videoAreaRef}>
                <video
                    ref={videoRef}
                    autoPlay
                    muted={true}
                    className={className(videoClass, { constrainedHeight })}
                />
                {overlayContainer}
            </div>
        )

    return (
        <div className='video-container' draggable={false}>
            {videoComponent}
            <div className="under-video-area">
                <UnderVideoButtons definition={definition}/>
            </div>
        </div>
    );
}

/**
 * Creates a single button for controlling the pan or tilt of the realsense camera
 *
 * @param props the direction of the button {@link PanTiltButton}
                */
const PanTiltButton = (props: { direction: ButtonPadButton }) => {
    let gridPosition: { gridRow: number, gridColumn: number };  // the position in the 3x3 grid around the video element
    let rotation: string;  // how to rotate the arrow icon to point in the correct direction
    const onClick = buttonFunctionProvider.provideFunctions(props.direction).onClick;

    // Specify button details based on the direction
    switch (props.direction) {
        case (ButtonPadButton.CameraTiltUp):
            gridPosition = { gridRow: 1, gridColumn: 2 };
            rotation = "-90";
            break;
        case (ButtonPadButton.CameraTiltDown):
            gridPosition = { gridRow: 3, gridColumn: 2 };
            rotation = "90";
            break;
        case (ButtonPadButton.CameraPanLeft):
            gridPosition = { gridRow: 2, gridColumn: 1 };
            rotation = "180";
            break;
        case (ButtonPadButton.CameraPanRight):
            gridPosition = { gridRow: 2, gridColumn: 3 };
            rotation = "0";  // by default the arrow icon points right
            break;
        default:
            throw Error(`unknown pan tilt button direction ${props.direction}`)
    }

    return (
        <button
            style={gridPosition}
            className={props.direction}
            onClick={onClick}
        >
            <span
                className="material-icons"
                style={{ transform: `rotate(${rotation}deg)` }}
            >arrow_right</span>
        </button>
    )
}

/******************************************************************************
 * Select context menu
 */

/** Props for {@link SelectContexMenu} */
type SelectContexMenuProps = {
    /** x and y location to render the context menu popup */
    clickXY: [number, number];
    /** Callback to select the video stream */
    selectSelf: () => void;
    /** Callback to select the child button pad */
    selectChild: () => void;
    /** Callback to hide the context menu popup when click outside */
    clickOut: () => void;
}

/**
 * Creates a context menu popup when user clicks during
 * customization mode so the user can choose between the button pad and its
 * parent video stream.
 *
 * @param props {@link SelectContexMenuProps}
                */
const SelectContexMenu = (props: SelectContexMenuProps) => {
    const ref = React.useRef<HTMLUListElement>(null);
    const [x, y] = props.clickXY;

    // Handler to close dropdown when click outside
    React.useEffect(() => {

        /** Closes context menu if user clicks outside */
        const handler = (e: any) => {
            // If didn't click inside the context menu or the existing SVG, then
            // hide the popup
            if (ref.current && !ref.current.contains(e.target)) {
                props.clickOut();
                console.log('clicked')
            }
        };
        window.addEventListener("click", handler, true);
        return () => {
            window.removeEventListener("click", handler);
        };
    }, []);

    /**
     * Handles when the user clicks on one of the context menu options
     * @param e mouse event of the click
     * @param self if true selects itself (a button pad), if false selects its 
     * parent (the video stream)
     */
    function handleClick(e: React.MouseEvent<HTMLLIElement>, self: boolean) {
        self ? props.selectSelf() : props.selectChild();

        // Make sure background elements don't receive a click
        e.stopPropagation();
    }

    return (

        <ul aria-label="Select"
            ref={ref}
            className="video-context-menu"
            style={{ top: `${y}px`, left: `${x}px` }}
        >
            <li onClick={(e) => handleClick(e, false)}>Button Pad</li>
            <li onClick={(e) => handleClick(e, true)}>Video Stream</li>
        </ul>
    );
}

/*******************************************************************************
 * Helper functions
 */

/**
 * Get the aspect ratio of the video based on the definition
 * 
 * @param definition definition of the video stream
 * @returns aspect ratio of the video stream
 */
function getVideoAspectRatio(definition: VideoStreamDef): number {
    switch (definition.id) {
        case (VideoStreamId.gripper):
            return gripperProps.width / gripperProps.height;
        case (VideoStreamId.overhead):
            return navigationProps.width / navigationProps.height;
        case (VideoStreamId.realsense):
            return realsenseProps.width / realsenseProps.height;
        default:
            throw Error(`undefined aspect ratio for ${definition.type}`)
    }
}

/**
 * Creates an overlay element for the video stream
 *
 * @param overlayDefinition definition for the component to overlay on the video stream
 * @param path path to the parent video stream component
 * @param sharedState {@link SharedState}
                        * @returns overlay element, or undefined if video stream doesn't have an overlay
                        */
function createOverlay(
    overlayDefinition: ComponentDefinition | undefined,
    path: string,
    sharedState: SharedState,
    aspectRatio: number): JSX.Element | undefined {
    // If overlay definition is undefined then there's no overlay for this stream
    if (!overlayDefinition) return undefined;
    if (!overlayDefinition.type) {
        console.warn(`Video stream at path ${path} has child with undefined type:`)
        console.warn(overlayDefinition);
        return undefined;
    }

    const overlayProps = {
        definition: overlayDefinition,
        path: path + "-0",
        sharedState: sharedState
    } as CustomizableComponentProps;

    switch (overlayDefinition.type) {
        case (ComponentType.ButtonPad):
            return <ButtonPad {...overlayProps} overlay aspectRatio={aspectRatio} />
        case (ComponentType.PredictiveDisplay):
            return <PredictiveDisplay {...overlayProps} />
        default:
            throw Error('Video stream at path ' + path + ' cannot overlay child of type' + overlayDefinition.type);
    }
}

/**
 * Gets the stream based on the identifier
 *
 * @param id identifier for the video stream
 * @param remoteStreams map of {@link RemoteStream}
                        * @returns the corresponding stream
                        */
function getStream(id: VideoStreamId, remoteStreams: Map<string, RemoteStream>): MediaStream {
    let streamName: string;
    switch (id) {
        case VideoStreamId.overhead:
            streamName = "overhead";
            break;
        case VideoStreamId.realsense:
            streamName = "realsense";
            break;
        case VideoStreamId.gripper:
            streamName = "gripper";
            break;
        default:
            throw Error(`unknow video stream id: ${id}`);
    }
    return remoteStreams.get(streamName)!.stream;
}

/**
 * Executes any functions required when this video stream is rendered. This might
 * be changing the view cropping for the overhead camera, hiding the depth sensing
 * for the Realsense, etc.
 * 
 * @param definition {@link VideoStreamDef}
 */
function executeVideoSettings(definition: VideoStreamDef) {
    switch (definition.id) {
        case (VideoStreamId.gripper):
            break;
        case (VideoStreamId.overhead):
            executeOverheadSettings(definition as OverheadVideoStreamDef);
            break;
        case (VideoStreamId.realsense):
            executeRealsenseSettings(definition as RealsenseVideoStreamDef);
            break;
        default:
            throw Error(`unknow video stream id: ${definition.id}`);
    }
}

/**
 * Executes functions to prepare for rendering the overhead video stream.
 * 
 * @param definition {@link OverheadVideoStreamDef}
 */
function executeOverheadSettings(definition: OverheadVideoStreamDef) {
    const overheadViewButton = definition.gripperView ? UnderVideoButton.GripperView : UnderVideoButton.DriveView;
    underVideoFunctionProvider.provideFunctions(overheadViewButton).onClick!();
}


/**
 * Executes functions to prepare for rendering the Realsense video stream.
 * 
 * @param definition {@link RealsenseVideoStreamDef}
 */
function executeRealsenseSettings(definition: RealsenseVideoStreamDef) {
    underVideoFunctionProvider.provideFunctions(UnderVideoButton.FollowGripper).onCheck!(definition.followGripper || false);
    underVideoFunctionProvider.provideFunctions(UnderVideoButton.DepthSensing).onCheck!(definition.depthSensing || false);
}

/*******************************************************************************
 * Under Video Buttons
 */

/**
 * Buttons to display under a video stream (e.g. toggle cropping of overhead 
 * stream, display depth sensing on Realsense, etc.)
 */
const UnderVideoButtons = (props: {definition: VideoStreamDef}) => {
    let buttons: JSX.Element | null;
    switch (props.definition.id) {
        case (VideoStreamId.gripper):
            buttons = null;
            break;
        case (VideoStreamId.overhead):
            buttons = <UnderOverheadButtons definition={props.definition}/>;
            break;
        case (VideoStreamId.realsense):
            buttons = <UnderRealsenseButtons definition={props.definition}/>;
            break;
        default:
            throw Error(`unknow video stream id: ${props.definition.id}`);
    }
    return buttons;
}

/**
 * Buttons to display under the overhead video stream.
 */
const UnderOverheadButtons = (props: {definition: OverheadVideoStreamDef}) => {
    const [gripperView, setGripperView] = React.useState<boolean>(props.definition.gripperView || false);
    const buttonText = "Switch to " + (props.definition.gripperView  ? "Drive View" : "Gripper View");

    /** Toggle the gripper view flag in the overhead definition */
    function handleClick() {
        console.log('click toggle overhead view', props.definition.gripperView, gripperView);
        props.definition.gripperView = !props.definition.gripperView;
        setGripperView(!gripperView);
        underVideoFunctionProvider.provideFunctions(props.definition.gripperView ? UnderVideoButton.GripperView : UnderVideoButton.DriveView).onClick!();
    }

    return (
        <button onClick={handleClick} >{buttonText}</button>
    )
}

/**
 * Buttons to display under the Realsense video stream.
 */
const UnderRealsenseButtons = (props: {definition: RealsenseVideoStreamDef}) => {
    const [rerender, setRerender] = React.useState<boolean>(false);

    return (
        <React.Fragment >
            {realsenseButtons.map(perspective =>
                <CameraPerspectiveButton perspective={perspective} key={perspective} />
            )}
            <CheckToggleButton
                checked={props.definition.followGripper || false}
                onClick={() => {
                    props.definition.followGripper = !props.definition.followGripper;
                    console.log(props.definition.followGripper)
                    setRerender(!rerender);
                    underVideoFunctionProvider.provideFunctions(UnderVideoButton.FollowGripper).onCheck!(props.definition.followGripper)
                }}
                label="Follow Gripper"
            />
            <CheckToggleButton
                checked={props.definition.depthSensing || false}
                onClick={() => {
                    props.definition.depthSensing = !props.definition.depthSensing;
                    setRerender(!rerender);
                    underVideoFunctionProvider.provideFunctions(UnderVideoButton.DepthSensing).onCheck!(props.definition.depthSensing)
                }}
                label="Depth Sensing"
            />
        </React.Fragment>
    )
}

/**
 * Button to change the camera perspective for a given video stream.
 */
const CameraPerspectiveButton = (props: {
    /** 
     * When the button is clicked, the corresponding video stream should change
     * to this perspective.
     */
    perspective: OverheadButtons | RealsenseButtons 
}) => {
    const onClick = underVideoFunctionProvider.provideFunctions(props.perspective).onClick;
    return (
        <button onClick={onClick}>
            {props.perspective}
        </button>
    )
}