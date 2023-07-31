import React, { useState, useEffect } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { RobotPose } from "shared/util";
import { Dropdown } from "../basic_components/Dropdown";
import "operator/css/PoseLibrary.css"
import "operator/css/basic_components.css"

export const PoseLibrary = (props: {
    savePose: (poseName: string, head: boolean, gripper: boolean, arm: boolean) => void,
    deletePose: (poseName: string) => void,
    savedPoseNames: () => string[],
    setRobotPose: (poseName: string) => void
}) => {
    const [showSavePoseModal, setShowSavePoseModal] = useState<boolean>(false);
    const [poses, setPoses] = useState<string[]>(props.savedPoseNames());
    const [selectedIdx, setSelectedIdx] = React.useState<number>();

    function getSelectedPoseName() {
        let poseName: string = "";
        if (selectedIdx == undefined) return;
        if (selectedIdx < poses.length) {
            poseName = poses[selectedIdx]
        }
        return poseName
    }

    function loadPose() {
        let poseName = getSelectedPoseName();
        if (!poseName) return;
        props.setRobotPose(poseName)
    }

    function deletePose() {
        let poseName = getSelectedPoseName();
        if (!poseName) return;
        props.deletePose(poseName)
        const index = poses.indexOf(poseName)
        poses.splice(index, 1)
        setPoses(poses => poses)
    }

    const SavePoseModal = (props: {
        savePose: (poseName: string, head: boolean, gripper: boolean, arm: boolean) => void,
        setShow: (show: boolean) => void,
        show: boolean
    }) => {
        const [name, setName] = React.useState<string>("");
        const [head, setHead] = React.useState<boolean>(false);
        const [gripper, setGripper] = React.useState<boolean>(false);
        const [arm, setArm] = React.useState<boolean>(false);
        function handleAccept() {
            if (name.length > 0) {
                if (!poses.includes(name)) {
                    setPoses(poses => [...poses, name])
                }
                props.savePose(name, head, gripper, arm);
            }
            setName("");
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                id="save-pose-modal"
                acceptButtonText="Save"
                acceptDisabled={name.length < 1}
            >
                <label htmlFor="new-pose-name"><b>Save Current Pose</b></label>
                <hr/>
                <div className="pose-name">
                    <label>Pose Name</label>
                    <input autoFocus type="text" id="new-pose-name" name="new-option-name"
                        value={name} onChange={(e) => setName(e.target.value)}
                        placeholder="Enter name"
                    />
                </div>
                <hr/>
                <div className="joint-checkbox">
                    <label>Joints to Save:</label>
                </div>
                <ul className="checkbox">
                    <li>
                        <input type="checkbox" id="head" name="save-head-pose" 
                            value="Head" onChange={(e) => setHead(e.target.checked)}
                        />
                        <label>Head</label>
                    </li>
                    <li>
                        <input type="checkbox" id="gripper" name="save-gripper-pose" 
                            value="Gripper" onChange={(e) => setGripper(e.target.checked)}
                        />
                        <label>Gripper</label>
                    </li>
                    <li>
                        <input type="checkbox" id="arm" name="save-arm-pose" 
                            value="Arm" onChange={(e) => setArm(e.target.checked)}
                        />                    
                        <label>Arm</label>
                    </li>
                </ul>
            </PopupModal>
        )
    }

    return (
        <React.Fragment>
            <div id="pose-library-container">Pose Library</div>
            <div id="pose-library-container">
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={poses}
                    placeholderText="Select a pose..."
                    placement="bottom"
                />
                <button className="play-btn" onClick={() => loadPose()}>
                    Play
                    <span className="material-icons">
                        play_circle
                    </span>
                </button>
                <button className="save-btn" onClick={() => setShowSavePoseModal(true)}>
                    Save
                    <span className="material-icons">
                        save
                    </span>
                </button>
                <button className="delete-btn" onClick={() => deletePose()}>
                    Delete
                    <span className="material-icons">
                        delete_forever
                    </span>
                </button>
            </div>
            <SavePoseModal 
                savePose={props.savePose}
                setShow={setShowSavePoseModal}
                show={showSavePoseModal}
            />
        </React.Fragment>
    )
}