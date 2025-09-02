import React, { useCallback, Dispatch, SetStateAction } from "react";
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft'; import "latest-createjs";
import "operator/css/Map.css";

interface MapProps {
    swipeableViewsIdxSet?: Dispatch<SetStateAction<number>>;
}

/**
 * Map component that displays a map for navigation.
 * It includes a back button to return to the previous view.
 *
 * @param {Dispatch<SetStateAction<number>>} swipeableViewsIdxSet - Set the index of the swipeable views.
 * @returns {JSX.Element} The rendered Map component.
 */

export const Map: React.FC<MapProps> = ({
    swipeableViewsIdxSet,
}) => {

    const goBack = useCallback(() => {
        swipeableViewsIdxSet(1);
    }, []);

    return (
        <React.Fragment>
            <div className="mobile-map-container">
                <div id="map" />
                <div className="back-button-wrapper">
                    <button
                        onClick={goBack}
                        className="btn btn-oval-ghost back-button"
                    >
                        <ChevronLeftIcon />
                    </button>
                </div>
            </div>
        </React.Fragment>
    );
};
