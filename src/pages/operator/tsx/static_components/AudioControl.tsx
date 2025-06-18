import "operator/css/AudioControl.css";
import { className, RemoteStream } from "shared/util";
import React from "react";
import VolumeOffIcon from "@mui/icons-material/VolumeOff";
import VolumeUpIcon from "@mui/icons-material/VolumeUp";

/**Props for {@link AudioControl} */
type AudioControlProps = {
    /** Remote robot video streams */
    remoteStreams: Map<string, RemoteStream>;
};

export const AudioControl = (props: AudioControlProps) => {
    // Create the audio element for listening to audio from the robot
    const audioRef = React.useRef<HTMLAudioElement>(null);

    // Keep track of whether the audio stream is muted or not.
    // NOTE: Audio must start muted for AutoPlay to work (see below)
    // https://developer.mozilla.org/en-US/docs/Web/Media/Autoplay_guide#autoplay_availability
    const [muted, setMuted] = React.useState(true);

    // Assign the source of the audio element to the audio stream from the robot
    const stream = React.useMemo(() => {
        if (props.remoteStreams.has("audio")) {
            return props.remoteStreams.get("audio")?.stream;
        } else {
            return null;
        }
    }, [props.remoteStreams]);
    React.useEffect(() => {
        if (!audioRef?.current) return;
        if (stream) {
            audioRef.current.srcObject = stream;
        }
    }, [audioRef.current, stream]);

    // Callback for when the user presses the button to toggle mute
    const toggleMute = React.useCallback(() => {
        setMuted((prev) => !prev);
    }, [setMuted]);

    return (
        <div className="audioControlContainer">
            <audio
                id="audio"
                ref={audioRef}
                className="audioControl"
                autoPlay
                muted={muted}
            >
                Robot audio playback is not supported on this browser
            </audio>
            <button className="button audioControlButton" onPointerDown={toggleMute}>
                {muted ? <VolumeOffIcon /> : <VolumeUpIcon />}
            </button>
        </div>
    );
};
