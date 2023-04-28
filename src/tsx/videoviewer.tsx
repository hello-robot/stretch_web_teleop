import React from "react"
import { VideoStream } from "./videostreams"

type VideoViewerProps = {
    /** The video stream object to display */
    videoStream: VideoStream,
    /** If the video stream needs to be rotated 90 degrees.zs */
    rotate: boolean,
    /** The button pad to overlay on this video. */
    buttonPad: React.ReactNode
}

/** Displays a single videostream and the button pad.
 * NOTE: this exists to handle the resizing logic for video streams which might
 * not be required. 
*/
const VideoViewer = (props: VideoViewerProps) => {
    /** The video stream to display */
    const videoStream: VideoStream = props.videoStream;

    /** Refrence to an indicator div which fills the available space to indicate
     * how to resize the image and button pad.
     */
    const indicatorDivRef = React.useRef<HTMLDivElement>(null);

    /** Style object for the child div containing the stream element */
    const [streamStyle, setStreamStyle] = React.useState({});
    const [displayWidth, setDisplayWidth] = React.useState(0);
    const [displayHeight, setDisplayHeight] = React.useState(0)

    // This handles resizing the video stream object to fit inside of the 
    if (props.rotate) {
        const heightRatio = videoStream.props.width / videoStream.props.height;
        const observer = new ResizeObserver(entries => {
            const { width, height } = entries[0].contentRect;
            const calculatedHeight = width * heightRatio;
            let newHeight: number;
            let newWidth: number;
            if (calculatedHeight < height) {
                newHeight = calculatedHeight;
                newWidth = width;
            } else {
                newHeight = height;
                newWidth = height / heightRatio;
            }
            const down = (newHeight - newWidth) / 2;
            const left = -(newHeight - width) / 2;
            const transform = `translate(${left}px, ${down}px) rotate(90deg)`
            const newStyle = {
                width: newHeight,
                height: newWidth,
                border: "1px dotted blue",
                transform: transform
            };
            setStreamStyle(newStyle);
            setDisplayWidth(newWidth);
            setDisplayHeight(newHeight);
        });
        React.useEffect(() => {
            if (!indicatorDivRef?.current) return;
            observer.observe(indicatorDivRef.current);
            return () => observer.disconnect();
        }, []);
    }
    // end of rotated resizing logic

    return (
        <div className="video-viewer">
            <div className="video-stream" style={streamStyle}>
                {videoStream.render()}
            </div>
            <div className="video-button-pad" style={{width: displayWidth, height: displayHeight}}>
                {props.buttonPad}
            </div>
            <div style={{width: "100%", height: "100%", background: "green"}} ref={indicatorDivRef}></div>
        </div>
    )
}