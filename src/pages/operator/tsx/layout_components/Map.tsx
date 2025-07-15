import React from "react";
import "latest-createjs";
import "operator/css/Map.css";

export const Map = () => {
    return (
        <React.Fragment>
            <div className="mobile-map-container">
                <div id="map" />
            </div>
        </React.Fragment>
    );
};
