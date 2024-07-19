import "operator/css/CustomizeButton.css";

type CustomizeButtonProps = {
    /** If the interface is in customization mode */
    customizing: boolean;
    /** Callback for clicking the button */
    onClick: () => void;
};

/** Button to toggle customization mode. */
export const CustomizeButton = (props: CustomizeButtonProps) => {
    const icon = props.customizing ? "check" : "build";
    const text = props.customizing ? "Done" : "Customize";
    return (
        <button
            onClick={props.onClick}
            id="customize-button"
            className={
                props.customizing ? "btn-turquoise font-white" : undefined
            }
        >
            <span className="material-icons">{icon}</span>
            {text}
        </button>
    );
};
// Uses icons from https://fonts.google.com/icons
