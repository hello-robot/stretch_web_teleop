import React, { useState, ReactElement, JSXElementConstructor } from "react";
import { MouseEvent } from "react";
import { className } from "shared/util";
import "operator/css/Alert.css";

// https://blog.logrocket.com/create-custom-react-alert-message/
export const Alert = (props: {
    children?: ReactElement<unknown, string | JSXElementConstructor<any>>;
    type: string;
    message?: string;
}) => {
    const [isShow, setIsShow] = useState(true);

    function renderElAlert() {
        return React.cloneElement(props.children!);
    }

    React.useEffect(() => {
        setIsShow(true);
    }, [props]);

    return (
        <div className={className("alert " + props.type, { hide: !isShow })}>
            <span className={"closebtn"} onClick={() => setIsShow(false)}>
                &times;
            </span>
            {props.children ? renderElAlert() : props.message}
        </div>
    );
};
