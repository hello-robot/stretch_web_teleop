import React from "react";
import { className, RemoteStream } from "shared/util";
import { CameraViewId } from "../utils/component_definitions";
import "operator/css/SimpleCameraView.css";
import { getIcon } from "../utils/svg";

/**
 * Displays a video stream with an optional button pad overlay
 *
 * @param props properties
 */
export const SimpleCameraView = (props: {
    id: CameraViewId;
    remoteStreams: Map<string, RemoteStream>;
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
        if (!videoRef?.current) return;
        videoRef.current.srcObject = stream;
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
        videoRef.current.style.marginTop = marginTop.toString() + "px";
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
            <>
                <div
                    className="simple-realsense"
                >
                    <div
                        className="simple-video-area"
                        style={{ gridRow: 2, gridColumn: 1 }}
                        ref={videoAreaRef}
                    >
                        <video
                            ref={videoRef}
                            autoPlay
                            muted={true}
                            disablePictureInPicture={true}
                            playsInline={true}
                            className="simple-video-canvas"
                            onPlay={() => setVideoSize(videoRef)}
                        />
                    </div>
                </div>
            </>
        ) : (
            <>
                <div
                    className="simple-video-area"
                    style={{ gridRow: 2, gridColumn: 1 }}
                    ref={videoAreaRef}
                >
                    <video
                        ref={videoRef}
                        autoPlay
                        muted={true}
                        disablePictureInPicture={true}
                        playsInline={true}
                        className="simple-video-canvas"
                        onPlay={() => setVideoSize(videoRef)}
                    />
                </div>
            </>
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
