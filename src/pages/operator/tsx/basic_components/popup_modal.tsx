import React from "react";
import "operator/css/basiccomponents.css"


export type PopupModalProps = {
    setShow: (show: boolean) => void,
    show: boolean,
    onAccept: () => void,
    id?: string
}

export const PopupModal: React.FunctionComponent<React.PropsWithChildren<PopupModalProps>> = (props) => {
    function handleClickAccept() {
        props.onAccept();
        props.setShow(false);
    }

    const element = props.show ? (
        <React.Fragment>
            <div id={props.id} className="popup-modal">
                {props.children}
                <div className="popup-modal-bottom-buttons">
                    <button className="btn-red" onClick={() => props.setShow(false)}>Cancel</button>
                    <button className="btn-green" onClick={handleClickAccept} style={{ float: "right" }}>Accept</button>
                </div>
            </div>
            <div onClick={() => props.setShow(false)} id="popup-background"></div>
        </React.Fragment>
    ) : null;

    return element;
}