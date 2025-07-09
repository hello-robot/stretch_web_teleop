import React, { Dispatch, SetStateAction, useState } from 'react';
import Modal from '../basic_components/ModalMobile';
import { AutoNavFunctions } from "./AutoNav";
import MagneticWrapper from '../static_components/MagneticWrapper';
import "operator/css/FooterAutoNav.css";

interface FooterAutoNavProps {
    isAutoNavHiddenSet: Dispatch<SetStateAction<boolean>>;
    handleSelectGoal: (selectGoal: boolean) => void;
    functs: AutoNavFunctions;
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    displayGoals: boolean;
    isModalAddLocationVisible: boolean;
    isModalAddLocationVisibleSet: Dispatch<SetStateAction<boolean>>;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
    isCurrentlyMoving: boolean;
    isCurrentlyMovingSet: Dispatch<SetStateAction<boolean>>;
    isSelectingGoal: boolean;
    isSelectingGoalSet: Dispatch<SetStateAction<boolean>>;
}

interface ModalAddLocationProps {
    functs: AutoNavFunctions;
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    displayGoals: boolean;
    isModalAddLocationVisible: boolean;
    isModalAddLocationVisibleSet: Dispatch<SetStateAction<boolean>>;
}

interface ModalLocationsMenuProps {
    // functs: AutoNavFunctions;
    poses: string[];
    // posesSet: Dispatch<SetStateAction<string[]>>;
    // displayGoals: boolean;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
}

const ModalAddLocation: React.FC<ModalAddLocationProps> = ({
    functs,
    poses,
    posesSet,
    displayGoals,
    isModalAddLocationVisibleSet,
    isModalAddLocationVisible,
}) => {
    const [locationName, locationNameSet] = React.useState<string>("");

    function handleAccept(): void {
        if (locationName.length > 0) {
            if (!poses.includes(locationName)) {
                posesSet((prevPoses) => [...prevPoses, locationName]);
            }
            functs.SaveGoal(locationName);
            functs.DisplayPoseMarkers(
                displayGoals,
                functs.GetSavedPoses(),
                functs.GetSavedPoseNames(),
                functs.GetSavedPoseTypes(),
            );
        }
        locationNameSet("");
        isModalAddLocationVisibleSet(false);
    }

    return (
        <Modal
            isOpen={isModalAddLocationVisible}
            onClose={() => isModalAddLocationVisibleSet(false)}
            title="Add Location"
            subtitle="AUTONAV"
            footer={
                <MagneticWrapper>
                    <button
                        disabled={locationName.length === 0}
                        className="btn btn-primary"
                        onClick={handleAccept}
                    >
                        Add Location
                    </button>
                </MagneticWrapper>
            }
        >
            <input
                autoFocus
                type="text"
                id="new-pose-name"
                name="new-option-name"
                className="input"
                value={locationName}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) => locationNameSet(e.target.value)}
                placeholder="Enter name of destination"
            />
        </Modal>
    );
};

const ModalLocationsMenu: React.FC<ModalLocationsMenuProps> = ({
    poses,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet,
}) => {

    function handleAccept(): void { }

    return (
        <Modal
            isOpen={isModalLocationsMenuVisible}
            onClose={() => isModalLocationsMenuVisibleSet(false)}
            title="Select Location"
            subtitle="AUTONAV"
        >
            <ul
                className="locations-menu-list"
                style={{
                    listStyle: "none",
                    margin: 0,
                    padding: 0,
                }}
            >
                {poses.map(pose => (
                    <li
                        className="locations-menu-list-item"
                        key={pose}
                        style={{
                            all: "unset",
                            display: "block",
                            cursor: "pointer",
                        }}
                    >
                        {pose}
                    </li>
                ))}
            </ul>
        </Modal>
    );
};

const FooterAutoNav: React.FC<FooterAutoNavProps> = ({
    isAutoNavHiddenSet,
    handleSelectGoal,
    functs,
    poses,
    posesSet,
    displayGoals,
    isModalAddLocationVisible,
    isModalAddLocationVisibleSet,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet,
    isCurrentlyMoving,
    isCurrentlyMovingSet,
    isSelectingGoal,
    isSelectingGoalSet,
}) => {

    const [selectedLocationMenuItemIdx, selectedLocationMenuItemIdxSet] = React.useState<number | -1>(-1);

    return (
        <div className="footer-auto-nav">

            {/* <LocationsMenu> */}
            <div className="locations-menu">
                <ModalLocationsMenu
                    poses={poses}
                    isModalLocationsMenuVisible={isModalLocationsMenuVisible}
                    isModalLocationsMenuVisibleSet={isModalLocationsMenuVisibleSet}
                />
                <button
                    onClick={() => {
                        isModalLocationsMenuVisibleSet(true);
                    }}
                >
                    📋
                </button>
            </div>
            {/* </LocationsMenu> */}

            {/* <AutoNavMainButton> */}
            {!isCurrentlyMoving
                ? (
                    <button
                        onPointerDown={() => {
                            // When selecting manually on map...
                            if (!isCurrentlyMoving && isSelectingGoal) {
                                console.log(1)
                                functs.Play();
                                isCurrentlyMovingSet(true);
                                isSelectingGoalSet(false);
                                functs
                                    .GoalReached()
                                    .then((goalReached) => isCurrentlyMovingSet(false));
                                // ...when selecting from Locations Menu
                            } else if (!isCurrentlyMoving && selectedLocationMenuItemIdx !== -1) {
                                console.log(2)
                                let pose: ROSLIB.Vector3 = functs.LoadGoal(selectedLocationMenuItemIdx)!;
                                functs.DisplayGoalMarker(pose);
                                functs.NavigateToAruco(selectedLocationMenuItemIdx);
                                isCurrentlyMovingSet(true);
                                isSelectingGoalSet(false);
                                functs
                                    .GoalReached()
                                    .then((goalReached) => isCurrentlyMovingSet(false));
                            }
                        }}
                        disabled={typeof functs.GetGoalPosition() === "number"}
                    >
                        <span>Play</span>
                    </button>
                )
                : (<div
                    className="mobile-map-cancel-btn"
                    onPointerDown={() => {
                        functs.CancelGoal();
                        isCurrentlyMovingSet(!isCurrentlyMoving);
                        isSelectingGoalSet(true);

                    }}
                >
                    <span>Cancel</span>
                    <span className="material-icons">cancel</span>
                </div>)}
            {/* </AutoNavMainButton> */}

            {/* <AddLocationButton> */}
            <div className="add-location">
                <ModalAddLocation
                    functs={functs}
                    poses={poses}
                    posesSet={posesSet}
                    displayGoals={displayGoals}
                    isModalAddLocationVisible={isModalAddLocationVisible}
                    isModalAddLocationVisibleSet={isModalAddLocationVisibleSet}
                />
                <button
                    onClick={() => {
                        isModalAddLocationVisibleSet(true);
                    }}
                >
                    Add
                </button>
            </div>
            {/* </AddLocationButton> */}
        </div >
    );
};

export default FooterAutoNav;
