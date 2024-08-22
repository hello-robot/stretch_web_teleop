import "operator/css/HomingButton.css";

type HomingButtonProps = {
    /** If the interface is in homing mode */
    homing: boolean;
    /** Callback for clicking the button */
    onClick: () => void;
};

/** Button to initiate robot homing. */
export const HomingButton = (props: HomingButtonProps) => {
    const text = props.homing ? "Homing..." : "Perform Homing";
    return (
        <button
            onClick={props.onClick}
            id="homing-button"
            className={
                props.homing ? "btn-turquoise font-white" : undefined
            }
        >
            {text}
        </button>
    );
};
// Uses icons from https://fonts.google.com/icons
