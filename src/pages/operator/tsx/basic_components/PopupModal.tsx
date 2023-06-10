import React from "react";
import "operator/css/basic_components.css"

/** Properties for {@link PopupModal} */
export type PopupModalProps = {
    /** If the popup should be shown */
    show: boolean,
    /** 
     * Callback to set if the popup is shown, used to close the popup on 
     * cancel, accept, or click outside of window.
     */
    setShow: (show: boolean) => void,
    /** Callback when the user clicks accept. */
    onAccept: () => void,
    /** Optional HTML id. */
    id?: string,
    /** Text to display on text button, defaults to "Accept" if undefined. */
    acceptButtonText?: string,
    /** Accept button disabled if true, enabled if false or undefined. */
    acceptDisabled?: boolean
}

/**
 * Generic component for a popup modal which covers the full screen with a 
 * darkened background.
 * @param props see {@link PopupModalProps}
 */
export const PopupModal: React.FunctionComponent<React.PropsWithChildren<PopupModalProps>> = (props) => {
    /** Call `onAccept` and hide the popup. */
    function handleClickAccept() {
        props.onAccept();
        props.setShow(false);
    }
    /** Handle user keyboard input. */
    function handleKeyDown(e: React.KeyboardEvent<HTMLDivElement>) {
        if (e.key == "Enter") {
            handleClickAccept();
        } else if (e.key == "Escape") {
            props.setShow(false);
        }
    }
    const element = props.show ? (
        <React.Fragment>
            <div 
                id={props.id} 
                className="popup-modal"
                onKeyDown={handleKeyDown}
            >
                {props.children}
                <div className="popup-modal-bottom-buttons">
                    <button className="btn-red" onClick={() => props.setShow(false)}>Cancel</button>
                    <button 
                        className="btn-turquoise font-white" 
                        onClick={handleClickAccept} 
                        style={{ float: "right" }}
                        disabled={props.acceptDisabled}
                    >
                        {props.acceptButtonText || "Accept"}
                    </button>
                </div>
            </div>
            <div onClick={() => props.setShow(false)} id="popup-background"></div>
        </React.Fragment>
    ) : null;

    return element;
}