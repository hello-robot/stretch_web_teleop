import React, { Dispatch, useRef, SetStateAction, useState, useCallback, useEffect } from 'react';
import ModalMobile from '../basic_components/ModalMobile';
import { AutoNavFunctions } from "./AutoNav";
import MagneticWrapper from '../static_components/MagneticWrapper';
import {
    ROSPoint,
} from 'shared/util';
import "operator/css/FooterAutoNav.css";
import { motion } from 'framer-motion';
import InputFluid from '../basic_components/InputFluid';
import SearchIcon from '@mui/icons-material/Search';
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft';
import ScrollableList from '../static_components/ScrollableList';
import DeleteIcon from '@mui/icons-material/Delete';
import StopCircleIcon from '@mui/icons-material/StopCircle';
import { set } from 'firebase/database';

interface FooterAutoNavProps {
    handleSelectGoal: (selectGoal: boolean) => void;
    functs: AutoNavFunctions;
    isModalAddLocationVisible: boolean;
    isModalAddLocationVisibleSet: Dispatch<SetStateAction<boolean>>;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
    isCurrentlyMoving: boolean;
    isCurrentlyMovingSet: Dispatch<SetStateAction<boolean>>;
    isSelectingGoal: boolean;
    isSelectingGoalSet: Dispatch<SetStateAction<boolean>>;
    selectedLocationMenuItem?: string;
    selectedLocationMenuItemSet: Dispatch<SetStateAction<string | undefined>>;
    goalPosition: ROSPoint | undefined; // Assuming goalPosition is a Vector3
    addToast: (type: "success" | "error" | "info", message: string, duration?: number) => void;
    swipeableViewsIdxSet?: Dispatch<SetStateAction<number>>;
}

interface ModalAddLocationProps {
    functs: AutoNavFunctions;
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    isModalAddLocationVisible: boolean;
    isModalAddLocationVisibleSet: Dispatch<SetStateAction<boolean>>;
    getPosesLatest: () => void;
    addToast: (type: "success" | "error" | "info", message: string, duration?: number) => void;
}

interface ModalLocationsMenuProps {
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    selectedLocationMenuItemSet: Dispatch<SetStateAction<string | undefined>>;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
    functs: AutoNavFunctions;
    getPosesLatest: () => void;
    addToast: (type: "success" | "error" | "info", message: string, duration?: number) => void;
}

/**
 * ModalAddLocation component allows users to add a new location 
 * by entering a name for the location.
 * 
 * @param functs - Functions for handling auto navigation.
 * @param poses - Current list of saved poses.
 * @param posesSet - Function to update the list of saved poses.
 * @param isModalAddLocationVisible - State to control visibility of the modal.
 * @param isModalAddLocationVisibleSet - Function to set visibility of the modal.
 * @param getPosesLatest - Function to fetch the latest poses.
 * @param addToast - Function to display toast notifications.
 */

const ModalAddLocation: React.FC<ModalAddLocationProps> = ({
    functs,
    poses,
    posesSet,
    isModalAddLocationVisibleSet,
    isModalAddLocationVisible,
    getPosesLatest,
    addToast,
}) => {

    const [locationName, locationNameSet] = React.useState<string>("");
    const closeModal = useCallback(() => isModalAddLocationVisibleSet(false), []);

    // Update poses in localStorage, and
    // update local state, "poses"
    function handleAccept(): void {
        if (locationName.length > 0) {
            if (!poses.includes(locationName)) {
                posesSet((prevPoses) => [...prevPoses, locationName]);
            }
            functs.SaveGoal(locationName);
            addToast('info', `Location "${locationName}" added.`);
            getPosesLatest();
        }
        locationNameSet("");
        isModalAddLocationVisibleSet(false);
    }

    const Footer = () => {
        return (
            <div className="footer-modal-add-location">
                <MagneticWrapper>
                    <button
                        disabled={locationName.length === 0}
                        className="btn btn-primary"
                        onClick={handleAccept}
                    >
                        Add Location
                    </button>
                </MagneticWrapper>
                <MagneticWrapper>
                    <button
                        className="btn btn-tertiary"
                        onPointerDown={closeModal}
                    >
                        Close
                    </button>
                </MagneticWrapper>
            </div>
        )
    }





    return (
        <ModalMobile
            isOpen={isModalAddLocationVisible}
            onClose={() => isModalAddLocationVisibleSet(false)}
            title="Add Location"
            subtitle="AUTONAV"
            footer={<Footer />}
        >
            <input
                autoFocus
                type="text"
                id="new-pose-name"
                name="new-option-name"
                className="input"
                value={locationName}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) => locationNameSet(e.target.value)}
                placeholder="Use a helpful name for this location"
                autoComplete="off"
            />
        </ModalMobile>
    );
};

/**
 * LocationsMenuListItem component represents a single item in the Locations Menu.
 * It allows users to edit or delete a saved location.
 * 
 * @param pose - The name of the location.
 * @param poses - Current list of saved poses.
 * @param posesSet - Function to update the list of saved poses.
 * @param functs - Functions for handling auto navigation.
 * @param getPosesLatest - Function to fetch the latest poses.
 * @param addToast - Function to display toast notifications.
 */

const LocationsMenuListItem: React.FC<{
    pose: string;
    poses: string[];
    posesSet: Dispatch<SetStateAction<string[]>>;
    selectedLocationMenuItemSet: Dispatch<SetStateAction<string | undefined>>;
    functs: AutoNavFunctions;
    getPosesLatest: () => void;
    addToast: (type: "success" | "error" | "info", message: string, duration?: number) => void;
    isModalLocationsMenuVisible: boolean;
    isModalLocationsMenuVisibleSet: Dispatch<SetStateAction<boolean>>;
}> = ({
    pose,
    poses,
    posesSet,
    selectedLocationMenuItemSet,
    functs,
    getPosesLatest,
    addToast,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet
}) => {

        const [poseNew, poseNewSet] = useState<string>("");
        const [isEditing, isEditingSet] = useState<boolean>(false);
        const [isSelected, isSelectedSet] = useState<boolean>(false);
        const inputRef = React.useRef<HTMLInputElement>(null);

        // Manage focus/blur for <InputFluid>
        useEffect(() => {
            if (isEditing && inputRef.current) {
                inputRef.current.focus();
            } else if (!isEditing) {
                inputRef.current.blur();
            }
        }, [isEditing]);

        // Reset to initial state
        // after closing modal
        useEffect(() => {
            if (!isModalLocationsMenuVisible) {
                poseNewSet("");
                isEditingSet(false);
                isSelectedSet(false);
                selectedLocationMenuItemSet(undefined);
            }
        }, [isModalLocationsMenuVisible]);

        const activateEditMode = (e: React.PointerEvent<HTMLButtonElement>) => {
            e.stopPropagation();
            console.log('hi')
            poseNewSet(pose);
            isEditingSet(true);
        };

        const handleSave = useCallback((e: React.PointerEvent<HTMLButtonElement>) => {
            console.log('ho')
            e.stopPropagation();
            // Update "pose" name if not already taken
            if (poseNew.length && !poses.includes(poseNew)) {
                functs.RenamePose(pose, poseNew);
                addToast('info', `Location "${pose}" renamed to "${poseNew}"`);
            }
            isEditingSet(false);
            getPosesLatest();
        }, [poseNew, poses, pose]);

        const handleDelete = useCallback((e: React.PointerEvent<HTMLButtonElement>) => {
            e.stopPropagation();
            functs.DeleteMapPose(pose);
            addToast('info', `Location "${pose}" deleted`);
            getPosesLatest();
        }, [functs, pose, addToast, getPosesLatest]);

        const onChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
            poseNewSet(e.target.value);
        }, []);

        // Handle when item
        // is selected
        const handleSelect = (poseName: string) => {
            isSelectedSet(true)
            addToast('info', `Selected "${poseName}"`);
            setTimeout(() => {
                selectedLocationMenuItemSet(poseName)
                isModalLocationsMenuVisibleSet(false);
            }, 1000);
        }

        return (
            <li
                className={`locations-menu-list-item ${isSelected ? 'selected' : ''}`}
                onClick={() => handleSelect(inputRef.current.value)}
            >
                <div className="locations-menu-list-item-left-column">
                    <InputFluid
                        inputRef={inputRef}
                        value={poseNew || pose}
                        onChange={onChange}
                        disabled={!isEditing}
                        onBlur={handleSave}
                        autoComplete="off"
                        classNameInput="locations-menu-list-item-input"
                    />
                    <button
                        className={`locations-menu-list-item-edit-button ${isEditing ? 'editing' : ''}`}
                        onClick={activateEditMode}
                    >
                        Edit
                    </button>
                </div>
                <div className="locations-menu-list-item-right-column">
                    {!isEditing
                        ? (
                            <button
                                onClick={handleDelete}
                                className="locations-menu-list-item-delete-button"
                            >
                                <DeleteIcon fontSize="small" />
                            </button>
                        )
                        : (
                            <button
                                onClick={handleSave}
                                className="btn btn-primary btn-sm"
                                disabled={!poseNew || poseNew.trim() === pose}
                            >
                                Save
                            </button>
                        )}
                </div>
            </li >
        );
    }

/**
 * ModalLocationsMenu component displays a list of saved locations
 * for auto navigation. Users can select a location to navigate to.
 * 
 * @param poses - List of saved poses for navigation goals.
 * @param posesSet - Function to update the list of saved poses.
 * @param functs - Functions for handling auto navigation.
 * @param isModalLocationsMenuVisible - State to control visibility of the modal.
 * @param isModalLocationsMenuVisibleSet - Function to set visibility of the modal.
 * @param getPosesLatest - Function to fetch the latest poses.
 * @param addToast - Function to display toast notifications.
 */

const ModalLocationsMenu: React.FC<ModalLocationsMenuProps> = ({
    poses,
    posesSet,
    selectedLocationMenuItemSet,
    functs,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet,
    getPosesLatest,
    addToast,
}) => {

    const [searchActive, setSearchActive] = useState<boolean>(false);
    const [searchTerm, setSearchTerm] = useState<string>("");
    const closeModal = useCallback(() => isModalLocationsMenuVisibleSet(false), []);

    const items = poses
        // Filter poses based on "searchTerm"...
        .filter((pose) => {
            return pose.toLowerCase().includes(searchTerm.toLowerCase());
        })
        // Reverse order to show
        // latest poses first...
        .reverse()
        // ...Map
        .map((pose) => (
            <LocationsMenuListItem
                key={pose}
                pose={pose}
                poses={poses}
                posesSet={posesSet}
                selectedLocationMenuItemSet={selectedLocationMenuItemSet}
                functs={functs}
                getPosesLatest={getPosesLatest}
                addToast={addToast}
                isModalLocationsMenuVisible={isModalLocationsMenuVisible}
                isModalLocationsMenuVisibleSet={isModalLocationsMenuVisibleSet}
            />
        ))

    // Reset search when activate/deactivated
    useEffect(() => {
        setSearchTerm("");
    }, [searchActive]);

    // Reset search when modal is closed
    useEffect(() => {
        if (!isModalLocationsMenuVisible) {
            setSearchActive(false);
            setSearchTerm("");
        }
    }, [isModalLocationsMenuVisible]);

    const Footer = () => (
        <MagneticWrapper>
            <button
                className="btn btn-tertiary"
                onPointerDown={closeModal}
            >
                Close
            </button>
        </MagneticWrapper>
    );

    const HeaderControls = () => (
        <div className={`locations-menu-search-controls ${searchActive ? 'active' : ''}`}>
            {!searchActive
                ? <button
                    className="locations-menu-search-btn"
                    onClick={() => setSearchActive(true)}
                    aria-label="Search locations"
                >
                    <SearchIcon />
                </button>
                : (
                    <div className="locations-menu-search-input-wrapper">
                        <button
                            className={`locations-menu-search-close-btn ${searchTerm.trim().length ? 'active' : ''}`}
                            onClick={() => setSearchActive(false)}
                            aria-label="Close search"
                        >
                            <ChevronLeftIcon />
                        </button>
                        <input
                            type="text"
                            className="locations-menu-search-input"
                            placeholder="Type to filter..."
                            value={searchTerm}
                            onChange={e => setSearchTerm(e.target.value)}
                            autoFocus
                        />
                    </div>
                )
            }
        </div>
    );

    return (
        <ModalMobile
            isOpen={isModalLocationsMenuVisible}
            onClose={closeModal}
            title="Select Location"
            subtitle="AUTONAV"
            HeaderControls={<HeaderControls />}
            footer={<Footer />}
        >
            <ScrollableList
                items={items}
                height={250}
                className="locations-menu-list"
            />
        </ModalMobile>
    );
};

/**
 * FooterAutoNav component provides a footer for auto navigation controls.
 * It includes buttons for starting navigation, adding locations,
 * and accessing the locations menu.
 * 
 * @param handleSelectGoal - Function to handle goal selection.
 * @param functs - Functions for handling auto navigation.
 * @param isModalAddLocationVisible - State to control visibility of the Add Location modal.
 * @param isModalAddLocationVisibleSet - Function to set visibility of the Add Location modal
 * @param isModalLocationsMenuVisible - State to control visibility of the Locations Menu modal.
 * @param isModalLocationsMenuVisibleSet - Function to set visibility of the Locations Menu modal
 * @param isCurrentlyMoving - State indicating if the robot is currently moving.
 * @param isCurrentlyMovingSet - Function to set the current moving state.
 * @param isSelectingGoal - State indicating if a goal is currently being selected.
 * @param isSelectingGoalSet - Function to set the goal selection state.
 * @param selectedLocationMenuItem - The name of the selected location menu item.
 * @param swipeableViewsIdxSet - Function to set the index of the swipeable views.
 * @param goalPosition - Current goal position for navigation.
 * @param addToast - Function to display toast notifications.
 */

const FooterAutoNav: React.FC<FooterAutoNavProps> = ({
    handleSelectGoal,
    functs,
    isModalAddLocationVisible,
    isModalAddLocationVisibleSet,
    isModalLocationsMenuVisible,
    isModalLocationsMenuVisibleSet,
    isCurrentlyMoving,
    isCurrentlyMovingSet,
    isSelectingGoal,
    isSelectingGoalSet,
    selectedLocationMenuItem,
    selectedLocationMenuItemSet,
    swipeableViewsIdxSet,
    goalPosition,
    addToast,
}) => {

    React.useEffect(() => {
        console.log(selectedLocationMenuItem)
        if (selectedLocationMenuItem) {
            let pose: ROSLIB.Transform = functs.LoadGoal(selectedLocationMenuItem)!;
            functs.DisplayGoalMarker(pose.translation);
        }
    }, [selectedLocationMenuItem]);

    // This function is called when the user
    // selects a goal on the map or from the
    // locations menu.
    const handleStartAutoNav = useCallback(() => {
        // ...when selecting from Locations Menu
        console.log(selectedLocationMenuItem, !isCurrentlyMoving, isSelectingGoal)
        if (!isCurrentlyMoving && selectedLocationMenuItem !== undefined) {
            let pose: ROSLIB.Transform = functs.LoadGoal(selectedLocationMenuItem)!;
            functs.NavigateToPose(pose)
            // functs.DisplayGoalMarker(pose.translation);
            isCurrentlyMovingSet(true);
            isSelectingGoalSet(false);
            selectedLocationMenuItemSet(undefined);
            functs
                .GoalReached()
                .then((goalReached) => {
                    isCurrentlyMovingSet(false)
                    isSelectingGoalSet(true);
                });
            // When selecting manually on map...
        } else if (!isCurrentlyMoving && isSelectingGoal) {
            functs.Play();
            isCurrentlyMovingSet(true);
            isSelectingGoalSet(false);
            functs
                .GoalReached()
                .then((goalReached) => {
                    isCurrentlyMovingSet(false)
                    isSelectingGoalSet(true);
                });
        }
    }, [functs, isCurrentlyMoving, isSelectingGoalSet, isSelectingGoal, selectedLocationMenuItem, isCurrentlyMovingSet]);

    // List of saved pose names for navigation goals
    const [poses, posesSet] = useState<string[]>(
        functs.GetSavedPoseNames(),
    );

    // Function to fetch the latest
    // pose names from localStorage
    // and update the local state, "poses".
    const getPosesLatest = useCallback(() => {
        // Fetch the latest pose names from the function provider...
        const poses = functs.GetSavedPoseNames();
        // Update local state with latest poses...
        posesSet(poses);
    }, []);

    return (
        <div className="footer-auto-nav">
            {/* <LocationsMenu> */}
            <div className="locations-menu-wrapper">
                <ModalLocationsMenu
                    poses={poses}
                    posesSet={posesSet}
                    selectedLocationMenuItemSet={selectedLocationMenuItemSet}
                    functs={functs}
                    isModalLocationsMenuVisible={isModalLocationsMenuVisible}
                    isModalLocationsMenuVisibleSet={isModalLocationsMenuVisibleSet}
                    getPosesLatest={getPosesLatest}
                    addToast={addToast}
                />
                <button
                    onPointerDown={() => {
                        isModalLocationsMenuVisibleSet(true);
                    }}
                    className="locations-menu"
                >
                    <span className="locations-menu-icon" />
                </button>
            </div>
            {/* </LocationsMenu> */}

            {/* <StartNavButton> */}
            {!isCurrentlyMoving
                ? (
                    <motion.button
                        onClick={handleStartAutoNav}
                        disabled={!goalPosition && !selectedLocationMenuItem}
                        className="auto-nav-button"
                        initial={false}
                        animate={goalPosition ? { width: 100 } : { width: 70 }}
                        transition={{
                            type: 'spring',
                            stiffness: 300,
                            damping: 20,
                            mass: 0.7,
                            bounce: 0.6,
                        }}
                        style={{ overflow: 'hidden', display: 'inline-flex', alignItems: 'center' }}
                    >
                        <span>Start</span>
                        <motion.span
                            className="auto-nav-button-icon"
                            initial={{ x: -40, opacity: 0, filter: 'brightness(1)' }}
                            animate={goalPosition
                                ? { x: 0, opacity: 1, filter: ['brightness(1)', 'brightness(1.7)', 'brightness(1)'] }
                                : { x: 20, opacity: 0, filter: 'brightness(1)' }
                            }
                            transition={goalPosition
                                ? {
                                    type: 'spring',
                                    stiffness: 400,
                                    damping: 12,
                                    mass: 0.6,
                                    bounce: 0.7,
                                    filter: {
                                        duration: 2,
                                        repeat: Infinity,
                                        repeatType: 'loop',
                                        ease: 'easeInOut',
                                    },
                                }
                                : {
                                    type: 'spring',
                                    stiffness: 400,
                                    damping: 12,
                                    mass: 0.6,
                                    bounce: 0.7,
                                }
                            }
                            style={{ display: 'inline-block', marginLeft: 8 }}
                        />
                    </motion.button>
                )
                : (<button
                    className="cancel-auto-nav-button"
                    onPointerDown={() => {
                        functs.CancelGoal();
                        isCurrentlyMovingSet(!isCurrentlyMoving);
                        isSelectingGoalSet(true);

                    }}
                >
                    <span>Stop</span>
                    <StopCircleIcon className="cancel-auto-nav-icon" />
                </button>)}
            {/* </AutoNavMainButton> */}

            {/* <AddLocationButton> */}
            <div className="add-location-wrapper">
                <ModalAddLocation
                    functs={functs}
                    poses={poses}
                    posesSet={posesSet}
                    isModalAddLocationVisible={isModalAddLocationVisible}
                    isModalAddLocationVisibleSet={isModalAddLocationVisibleSet}
                    getPosesLatest={getPosesLatest}
                    addToast={addToast}
                />
                <button
                    onPointerDown={() => {
                        isModalAddLocationVisibleSet(true);
                    }}
                    className="add-location"
                >
                    <span className="add-location-icon" />
                </button>
            </div>
            {/* </AddLocationButton> */}
        </div >
    );
};

export default FooterAutoNav;
