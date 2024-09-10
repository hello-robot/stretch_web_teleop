import React, { useState, ReactElement, JSXElementConstructor } from "react";
import { MouseEvent } from "react";
import { className } from "shared/util";
import "operator/css/Alert.css";

// https://blog.logrocket.com/create-custom-react-alert-message/
export const Alert = (props: {
    children?: ReactElement<unknown, string | JSXElementConstructor<any>>;
    type: string;
    message?: string;
    hide_close_button?: boolean;
    style?: React.CSSProperties;
}) => {
    const [isShow, setIsShow] = useState(true);

    function renderElAlert() {
        return React.cloneElement(props.children!);
    }

    React.useEffect(() => {
        setIsShow(true);
    }, [props]);

    return (
        <div
            className={className("alert " + props.type, { hide: !isShow })}
            style={props.style}
        >
            <span
                className={className("closebtn", {
                    hide: props.hide_close_button,
                })}
                onClick={() => setIsShow(false)}
            >
                &times;
            </span>
            {props.children ? renderElAlert() : props.message}
        </div>
    );
};
