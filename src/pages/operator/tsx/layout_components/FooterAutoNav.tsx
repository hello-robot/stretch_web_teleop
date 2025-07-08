import React, { Dispatch, SetStateAction } from 'react';
import Modal from '../basic_components/ModalMobile';
import { AutoNavFunctions } from "./AutoNav";
import MagneticWrapper from '../static_components/MagneticWrapper';
import "operator/css/FooterAutoNav.css";

interface FooterControlsProps {
    isAutoNavHiddenSet: Dispatch<SetStateAction<boolean>>;
    selectGoal2: boolean;
    handleSelectGoal: (selectGoal: boolean) => void;
    selectGoal2Set: Dispatch<SetStateAction<boolean>>;
    functs: AutoNavFunctions;
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    displayGoals: boolean;
    isModalAddLocationVisible: boolean;
    isModalAddLocationVisibleSet: Dispatch<SetStateAction<boolean>>;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
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

const FooterAutoNav: React.FC<FooterControlsProps> = ({
    isAutoNavHiddenSet,
    handleSelectGoal,
    selectGoal2,
    selectGoal2Set,
    functs,
    poses,
    posesSet,
    displayGoals,
    isModalAddLocationVisible,
    isModalAddLocationVisibleSet,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet,
}) => {
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

            <button>Move</button>

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
        </div>
    );
};

export default FooterAutoNav;
