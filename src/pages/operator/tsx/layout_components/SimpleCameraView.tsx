import React from "react";
import { className, RemoteStream } from "shared/util";
import { buttonFunctionProvider } from "..";
import { CameraViewId } from "../utils/component_definitions";
import { ButtonPadButton, ButtonState, panTiltButtons } from "../function_providers/ButtonFunctionProvider";
import "operator/css/SimpleCameraView.css"
import { getIcon } from "../utils/svg";

/**
 * Displays a video stream with an optional button pad overlay
 * 
 * @param props properties
 */
export const SimpleCameraView = (props: { id: CameraViewId, remoteStreams: Map<string, RemoteStream> }) => {
    // Reference to the video element
    const videoRef = React.useRef<HTMLVideoElement>(null);
    // Get the stream to display inside the video
    const stream: MediaStream = getStream(props.id, props.remoteStreams);
    // Refrence to the div immediately around the video element
    const videoAreaRef = React.useRef<HTMLDivElement>(null);
    // Boolean representing if the video stream needs to be constrained by height
    // (constrained by width otherwise)
    const [constrainedHeight, setConstrainedHeight] = React.useState<boolean>(false);

    // Update the source of the video stream 
    React.useEffect(() => {
        if (!videoRef?.current) return;
        videoRef.current.srcObject = stream;
    }, [stream]);


    // Constrain the width or height when the stream gets too large
    React.useEffect(() => {
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

    const videoComponent = (props.id === CameraViewId.realsense) ?
        (
            <>
                <div className={className("simple-realsense", { constrainedHeight })}>
                    {panTiltButtons.map(dir => <PanTiltButton direction={dir} key={dir} />)}
                    <div className="simple-video-area" style={{ gridRow: 2, gridColumn: 1 }} ref={videoAreaRef}>
                        <video
                            ref={videoRef}
                            autoPlay
                            muted={true}
                            className={className('simple-video-canvas', { constrainedHeight })}
                        />
                    </div>
                </div>
            </>
        )
        :
        (
            <>
                <div className="simple-video-area" style={{ gridRow: 2, gridColumn: 1 }} ref={videoAreaRef}>
                    <video
                        ref={videoRef}
                        autoPlay
                        muted={true}
                        className={className('simple-video-canvas', { constrainedHeight })}
                    />
                </div>
            </>
        )

    return (
        <div className='simple-video-container' draggable={false}>
            {videoComponent}
        </div>
    );
}

/**
 * Creates a single button for controlling the pan or tilt of the realsense camera
 *
 * @param props the direction of the button {@link PanTiltButton}
                */
const PanTiltButton = (props: { direction: ButtonPadButton }) => {
    const functs = buttonFunctionProvider.provideFunctions(props.direction);
    const dir = props.direction.split(" ")[2]
    let rotation: string;

    // Specify button details based on the direction
    switch (props.direction) {
        case (ButtonPadButton.CameraTiltUp):
            rotation = "-90";
            break;
        case (ButtonPadButton.CameraTiltDown):
            rotation = "90";
            break;
        case (ButtonPadButton.CameraPanLeft):
            rotation = "180";
            break;
        case (ButtonPadButton.CameraPanRight):
            rotation = "0";  // by default the arrow icon points right
            break;
        default:
            throw Error(`unknown pan tilt button direction ${props.direction}`)
    }

    return (
        <button
            className={'overlay btn-' + dir}
            onPointerDown={functs.onClick}
            onPointerUp={functs.onRelease}
            onPointerLeave={functs.onLeave}
        >
            {/* <img height={100} width={100} src={getIcon(props.direction)} className={ButtonState.Inactive} /> */}
            <span
                className="material-icons icon"
                style={{ transform: `rotate(${rotation}deg)` }}
            >arrow_right</span>
        </button>
    )
}

/*******************************************************************************
 * Helper functions
 */

/**
 * Gets the stream based on the identifier
 *
 * @param id identifier for the video stream
 * @param remoteStreams map of {@link RemoteStream}
                        * @returns the corresponding stream
                        */
function getStream(id: CameraViewId, remoteStreams: Map<string, RemoteStream>): MediaStream {
    let streamName: string;
    switch (id) {
        case CameraViewId.overhead:
            streamName = "overhead";
            break;
        case CameraViewId.realsense:
            streamName = "realsense";
            break;
        case CameraViewId.gripper:
            streamName = "gripper";
            break;
        default:
            throw Error(`unknow video stream id: ${id}`);
    }
    return remoteStreams.get(streamName)!.stream;
}

