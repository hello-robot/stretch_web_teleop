import React, { useState } from "react";
import Modal from '../basic_components/ModalMobile';
import MagneticWrapper from "../static_components/MagneticWrapper";
import "operator/css/ActionMode.css";
import { ActionModeType } from "../utils/component_definitions";

/**Details of the action modes */
type ActionModeDetails = {
    /**Name of the action mode */
    mode: ActionModeType;
    /**The description for the action mode */
    desc: string;
};

/**Props for {@link ActionSpeed} */
export type ActionModeProps = {
    /** Initial action mode when interface first loaded. */
    mode: ActionModeType;

    /**
     * Callback function when a new action mode is selected.
     * @param newMode the new selected action mode
     */
    onChange: (newMode: ActionModeType) => void;

    /**
     * Whether or not the camera veil is currently displayed.
     */
    isCameraVeilVisible?: boolean;

    /**
     * Callback function to display the camera veil when the action speed modal is open.
     * @param enable whether or not to display the camera veil
     */
    setCameraVeilCallback: (enable: boolean) => void;

};

/**
 * Set of buttons so the user can control the scaling of the speed for all controls.
 * @param props see {@link ActionModeProps}
 */
export const ActionMode = (props: ActionModeProps) => {
    const [isModalOpen, setIsModalOpen] = React.useState<boolean>(false);

    return (
        <div className="action-mode">
            <ModalActionMode isOpen={isModalOpen} mode={props.mode} handleClose={(newActionMode: ActionModeType) => {
                setIsModalOpen(false);
                props.setCameraVeilCallback(false);
                props.onChange(newActionMode)
            }} />
            <MagneticWrapper>
                <button
                    className="button-action-mode"
                    onClick={() => {
                        setIsModalOpen(!isModalOpen);
                        props.setCameraVeilCallback(!isModalOpen)
                    }}
                    aria-label="Change action mode"
                    aria-hidden={props.isCameraVeilVisible}
                >
                    <span className="action-mode-icon"></span>
                    <div>{props.mode}</div>
                </button>
            </MagneticWrapper>
        </div>
    );
};

interface ModalActionModeProps {
    mode: ActionModeType;
    isOpen: boolean;
    /**
     * Function callback for execution on modal close
     * @param newActionMode the newly selected action mode
     */
    handleClose: (newActionMode: ActionModeType) => void;
}

const ModalActionMode: React.FC<ModalActionModeProps> = ({ mode, isOpen, handleClose }) => {
    const [selectedMode, setSelectedMode] = useState<ActionModeType>(mode);
    const [loading, setLoading] = useState<boolean>(false);

    const options: ActionModeDetails[] = [
        { mode: ActionModeType.PressAndHold, desc: 'Stretch moves while you press and hold a button and stops when you release.' },
        { mode: ActionModeType.ClickClick, desc: 'Stretch moves when you tap a button and stops when you tap again.' },
        { mode: ActionModeType.StepActions, desc: 'Stretch moves a fixed distance based on the selected speed with each click.' },
    ];

    const handleConfirm = (): void => {
        if (loading) return;
        setLoading(true);
        // This is a synthetic delay, replace with actual logic
        console.log("Selected Action Mode:", selectedMode);
        setTimeout(() => {
            setLoading(false);
            handleClose(selectedMode); // Close the modal
            // Potentially call a prop to update the action mode in the parent component
            // e.g., props.onModeChange(selectedMode);
        }, 0);
    };

    const modalFooterContent = (
        <MagneticWrapper>
            <button
                className="btn btn-primary" // Style from ModalActionMode.css
                onClick={handleConfirm}
                disabled={loading}
            >
                {loading ? <span className="spinner" /> : 'Confirm'}
            </button>
        </MagneticWrapper>
    );

    return (
        <Modal
            isOpen={isOpen}
            onClose={() => handleClose}
            title="Action Mode"
            subtitle="SELECT"
            footer={modalFooterContent}
            modalClassName="action-mode-modal"
        >
            <div className="action-mode-options">
                {options.map(opt => (
                    <label
                        key={opt.mode}
                        className={`radio-group ${selectedMode === opt.mode ? 'selected' : ''}`}
                    >
                        <input
                            type="radio"
                            name="actionMode"
                            value={opt.mode}
                            checked={selectedMode === opt.mode}
                            onChange={() => setSelectedMode(opt.mode)}
                            disabled={loading}
                            aria-label={`Use \"${opt.mode}\" `}
                            aria-checked={selectedMode === opt.mode}
                        />
                        <span className="radio-label" aria-hidden="true"
                        >{opt.mode}</span>
                        {selectedMode === opt.mode && (
                            <p className="radio-desc" id={`desc-${opt.mode}`} aria-hidden="true">{opt.desc}</p>
                        )}
                    </label>
                ))}
            </div>
        </Modal>
    );
};