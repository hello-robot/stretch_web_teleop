import React from "react";
import { className, RemoteStream, StretchModel } from "shared/util";
import { stretchModel } from '..'
import { CameraViewId } from "../utils/component_definitions";
// import DeviceVisualOverlay from "../static_components/DeviceVisualOverlay";
import {
    panTiltButtons,
} from "../function_providers/ButtonFunctionProvider";
import { PanTiltButton } from "./PanTiltButton";
import "operator/css/SimpleCameraView.css";

/**
 * Displays a video stream with an optional button pad overlay
 *
 * @param props properties
 */
export const SimpleCameraView = (props: {
    id: CameraViewId;
    remoteStreams: Map<string, RemoteStream>;
    isCameraVeilVisible: boolean;
}) => {
    // Reference to the video element
    const videoRef = React.useRef<HTMLVideoElement>(null);
    // Get the stream to display inside the video
    const stream: MediaStream = getStream(props.id, props.remoteStreams);
    // Reference to the div immediately around the video element
    const videoAreaRef = React.useRef<HTMLDivElement>(null);
    // Boolean representing if the video stream needs to be constrained by height
    // (constrained by width otherwise)
    const [constrainedHeight, setConstrainedHeight] =
        React.useState<boolean>(false);

    // Update the source of the video stream
    React.useEffect(() => {

        const video = videoRef.current;

        if (!video) return;

        // Patch in the video stream!
        video.srcObject = stream;

        // If GripperCam's video is low-res
        // then scale up to "videoResolutionWidthMaximum"
        if (props.id === CameraViewId.gripper) {
            const handleResize = () => {
                const videoResolutionWidth = video.videoWidth
                const videoResolutionWidthMaximum = 1000;

                if (videoResolutionWidth === videoResolutionWidthMaximum) {
                    video.style.scale = '1';
                } else {
                    video.style.scale = (videoResolutionWidthMaximum / videoResolutionWidth).toString();
                }
            };

            video.addEventListener("resize", handleResize);
        }

    }, [stream]);

    function setVideoSize(videoRef) {
        const videoRect = videoRef.current.getBoundingClientRect();
        let marginTop = 0;
        let min_control_panel_size = 0; // in pixels
        marginTop = Math.min(
            window.innerHeight - min_control_panel_size - videoRect.height,
            0,
        );
        document
            .querySelector(".btn-down")
            ?.setAttribute(
                "style",
                "margin-top:" +
                (videoRect.height + marginTop - 70).toString() +
                "px;",
            );
        document
            .querySelector(".depth-sensing")
            ?.setAttribute(
                "style",
                "margin-top:" +
                (videoRect.height + marginTop - 42).toString() +
                "px;",
            );
    }

    // Constrain the width or height when the stream gets too large
    React.useEffect(() => {
        const resizeObserver = new ResizeObserver((entries) => {
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

    const videoComponent =
        props.id === CameraViewId.realsense ||
            props.id === CameraViewId.overhead ? (
            <div
                className="simple-headcam"
            >
                {/* <DeviceVisualOverlay /> */}
                <div
                    className="simple-headcam-area"
                    style={{ gridRow: 2, gridColumn: 1 }}
                    ref={videoAreaRef}
                >
                    {
                        // If Stretch3 or earlier && Realsense camera, show pan-tilt overlay
                        stretchModel === StretchModel.SE3
                            ? <div
                                className={className("realsense-pan-tilt-grid", {
                                    constrainedHeight,
                                })}
                            >
                                {panTiltButtons.map((dir) => (
                                    <PanTiltButton direction={dir} key={dir} />
                                ))}
                            </div>
                            : null
                    }
                    <video
                        ref={videoRef}
                        autoPlay
                        muted={true}
                        disablePictureInPicture={true}
                        playsInline={true}
                        className="simple-headcam-canvas"
                        onPlay={() => setVideoSize(videoRef)}
                    />
                </div>
            </div>
        ) : (
            <div
                className="simple-grippercam"
            >
                <div
                    className="simple-grippercam-area"
                    style={{ gridRow: 2, gridColumn: 1 }}
                    ref={videoAreaRef}
                >
                    <video
                        ref={videoRef}
                        autoPlay
                        muted={true}
                        disablePictureInPicture={true}
                        playsInline={true}
                        className="simple-grippercam-canvas"
                        onPlay={() => setVideoSize(videoRef)}
                    />
                </div>
            </div>
        );

    return (
        <div className="simple-video-container" draggable={false}>
            {videoComponent}
        </div>
    );
};

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
function getStream(
    id: CameraViewId,
    remoteStreams: Map<string, RemoteStream>,
): MediaStream {
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
