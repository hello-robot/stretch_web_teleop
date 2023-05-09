
type CustomizeButtonProps = {
    /** If the interface is in customization mode */
    customizing: boolean;
    /** Callback for clicking the button */
    onClick: () => void;
}


/** Button to toggle customization mode. */
export const CustomizeButton = (props: CustomizeButtonProps) => {
    const icon = props.customizing ? "check_circle" : "build_circle";
    return (
        <span
            className="material-icons"
            style={{ fontSize: "36px", cursor: "pointer" }}
            onClick={props.onClick}
        >
            {icon}
        </span>
    )
}
// Uses icons from https://fonts.google.com/icons