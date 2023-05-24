import React from "react";
import { className, gripperProps, navigationProps, realsenseProps, RemoteStream } from "shared/util";
import { VideoStreamDef, ComponentType, VideoStreamId, ComponentDefinition } from "../utils/componentdefinitions";
import { ButtonPad } from "./buttonpads";
import { CustomizableComponentProps, SharedState } from "./customizablecomponent";
import { DropZone } from "./dropzone";
import { PredictiveDisplay } from "./predictivedisplay";
import "operator/css/videostreamcomponent.css"

/** Array of the different directions for the pan tilt buttons */
const panTiltButtonDirections: string[] = ['up', 'down', 'left', 'right'];
/** Type to specify the different directions for the pan tilt buttons */
export type PanTiltButtonDirection = typeof panTiltButtonDirections[number];

/**
 * Displays a video stream with an optional button pad overlay
 * 
 * @param props properties
 */
export const VideoStreamComponent = (props: CustomizableComponentProps) => {
    // Dimensions for the overlay so its the same width and height as the video
    const [overlayDimensions, setOverlayDimensions] = React.useState({});
    const [videoWidth, setVideoWidth] = React.useState(100);
    // Reference to the video element
    const videoRef = React.useRef<HTMLVideoElement>(null);
    const overlayContainerRef = React.useRef<HTMLDivElement>(null);
    // X and Y position of the cursor when user clicks on the video
    const [clickXY, setClickXY] = React.useState<[number, number] | null>(null);
    const definition = props.definition as VideoStreamDef;
    if (!definition.children) throw Error('Video Stream definition should have children');
    // Get the stream to display in the video
    const stream: MediaStream = getStream(definition.id, props.sharedState.remoteStreams);

    // Create the overlay
    const overlayDefinition = definition.children.length > 0 ? definition.children[0] : undefined;
    const overlay = createOverlay(overlayDefinition, props.path, props.sharedState);

    const videoAspectRatio = getVideoAspectRatio(definition);

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

    const overlayContainer = (
        <div
            className={className("video-overlay-container", { customizing, selected })}
            // style={overlayDimensions}
            ref={overlayContainerRef}
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

    return (
        <div className='video-container' draggable={false}>
            <SubVideoComponent {...{ videoWidth, definition, videoRef, videoClass, overlayContainer }} />
            <div className="under-video-area">
                {getUnderVideoButtons(definition)}
            </div>
        </div>
    );
}

/** Properties for {@link SubVideoComponent} */
type SubVideoComponentProps = {
    definition: VideoStreamDef,
    videoRef: React.RefObject<HTMLVideoElement>,
    videoWidth: number,
    videoClass: string,
    overlayContainer: JSX.Element
}

/**
 * The actual video element, either a plain HTML video element or a div with 
 * the pan tilt controls for the realsense if the video stream is of the realsense
 * 
 * @param props {@link SubVideoComponentProps}
 */
const SubVideoComponent = (props: SubVideoComponentProps) => {
    const videoAreaRef = React.useRef<HTMLDivElement>(null);
    const [constrainedHeight, setConstrainedHeight] = React.useState<boolean>(false);
    const videoRef = props.videoRef;
    const buttons = panTiltButtonDirections.map(dir => <PanTiltButton direction={dir} key={dir} />)

    // Constrain the width or height when the stream gets too large
    React.useEffect(() => {
        console.log('use effect')
        const resizeObserver = new ResizeObserver(entries => {
            console.log('resize function')

            // height and width of overlay container
            const { height, width } = entries[0].contentRect;

            // height and width of video stream
            if (!videoRef?.current) return;
            const videoRect = videoRef.current.getBoundingClientRect();
            
            if (videoRect.height > height) {
                setConstrainedHeight(true);
                return;
            } 
            if (videoRect.width > width) {
                setConstrainedHeight(false);
            }
        });
        if (!videoAreaRef?.current) return;
        resizeObserver.observe(videoAreaRef.current);
        return () => resizeObserver.disconnect();
    }, []);

    if (props.definition.id === VideoStreamId.realsense) {
        return (
            <div className={className("realsense-pan-tilt-grid", { constrainedHeight })}>
                {buttons}

                <div className="video-area" style={{ gridRow: 2, gridColumn: 2 }} ref={videoAreaRef}>
                    <video
                        ref={props.videoRef}
                        autoPlay
                        muted={true}
                        className={className(props.videoClass, { constrainedHeight })}
                    />
                    {props.overlayContainer}
                </div>
            </div>
        )
    } else {
        return (
            <div className="video-area" style={{ gridRow: 2, gridColumn: 1 }} ref={videoAreaRef}>
                <video
                    ref={props.videoRef}
                    autoPlay
                    muted={true}
                    className={className(props.videoClass, { constrainedHeight })}
                />
                {props.overlayContainer}
            </div>
        )
    }
}


// TODO: implement video stream pan tilt function provider
const mockFunctionProvider = (direction: PanTiltButtonDirection): () => void => {
    return () => console.log('clicked pan tilt button', direction)
}

/**
 * Creates a single button for controlling the pan or tilt of the realsense camera
 *
 * @param props the direction of the button {@link PanTiltButtonDirection}
                */
const PanTiltButton = (props: { direction: PanTiltButtonDirection }) => {
    let gridPosition: { gridRow: number, gridColumn: number };  // the position in the 3x3 grid around the video element
    let rotation: string;  // how to rotate the arrow icon to point in the correct direction
    const onClick = mockFunctionProvider(props.direction);  // TODO: implement video stream pan tilt function provider

    // Specify button details based on the direction
    switch (props.direction) {
        case ('up'):
            gridPosition = { gridRow: 1, gridColumn: 2 };
            rotation = "-90";
            break;
        case ('down'):
            gridPosition = { gridRow: 3, gridColumn: 2 };
            rotation = "90";
            break;
        case ('left'):
            gridPosition = { gridRow: 2, gridColumn: 1 };
            rotation = "180";
            break;
        case ('right'):
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
 **************************************************************************** */

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
 ******************************************************************************/

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
    sharedState: SharedState): JSX.Element | undefined {
    // If overlay definition is undefined then there's no overlay for this stream
    if (!overlayDefinition) return undefined;

    const overlayProps = {
        definition: overlayDefinition,
        path: path + "-0",
        sharedState: sharedState
    } as CustomizableComponentProps;

    switch (overlayDefinition?.type) {
        case (ComponentType.ButtonPad):
            return <ButtonPad {...overlayProps} overlay />
        case (ComponentType.PredictiveDisplay):
            return <PredictiveDisplay {...overlayProps} />
        default:
            throw Error('Video stream at path ' + path + ' cannot overlay child ' + overlayDefinition);
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

function getUnderVideoButtons(definition: VideoStreamDef): JSX.Element | undefined {
    let buttons: JSX.Element | undefined;
    switch (definition.id) {
        case (VideoStreamId.gripper):
            buttons = undefined;
            break;
        case (VideoStreamId.overhead):
            buttons = getOverheadButtons();
            break;
        case (VideoStreamId.realsense):
            buttons = getRealsenseButtons();
            break;
        default:
            throw Error(`unknow video stream id: ${definition.id}`);
    }
    return buttons;
}

function getOverheadButtons() {
    return (
        <React.Fragment >
            <button>Crop to arm</button>
            <button>Remove crop</button>
        </React.Fragment>
    )
}

function getRealsenseButtons() {
    return (
        <React.Fragment >
            <button>Look at gripper</button>
            <button>Look at base</button>
            <ToggleFollowGripperButton />
        </React.Fragment>
    )
}

const ToggleFollowGripperButton = () => {
    const [followGripper, setFollowGripper] = React.useState<boolean>(false);
    return (
        <CheckToggleButton
            checked={followGripper}
            onClick={() => setFollowGripper(!followGripper)}
            label={"Follow gripper"}
        />
    )
}

type CheckToggleButtonProps = {
    checked: boolean,
    onClick: () => void,
    label: string,
}

const CheckToggleButton = (props: CheckToggleButtonProps) => {
    const { checked } = props;
    const icon = checked ? "check_box" : "check_box_outline_blank";
    return (
        <button
            className={className("check-toggle-button", { checked })}
            onClick={props.onClick}
        >
            <span className={"material-icons"}>{icon}</span>
            Follow gripper
        </button>
    )
}