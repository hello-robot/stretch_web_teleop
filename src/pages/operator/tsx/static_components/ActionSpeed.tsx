import React, { useState } from "react";
import Modal from '../basic_components/ModalMobile';
import "operator/css/ActionSpeed.css";
import { buttonFunctionProvider } from "..";

/**Details of a velocity setting */
type ActionSpeedDetails = {
    /**Name of the setting to display on the button */
    label: string;
    /**The speed of this setting */
    speed: number;
};

/**Props for {@link ActionSpeed} */
type ActionSpeedProps = {
    /** Initial speed when interface first loaded. */
    speed: number;

    /**
     * Callback function when a new speed is selected.
     * @param newSpeed the new selected speed
     */
    onChange: (newScale: number) => void;

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
 * The different velocity settings to display.
 * Scale: 0 -> 1.6
 */
export const VELOCITY_SCALE: ActionSpeedDetails[] = [
    { label: "slow", speed: 0.75 },
    { label: "medium", speed: 1.25 },
    { label: "fast", speed: 2.0 },
];

const getSpeedByLabel = (label: string): number | undefined => {
    return VELOCITY_SCALE.find(item => item.label === label)?.speed;
};

const getLabelBySpeed = (speed: number): string | undefined => {
    return VELOCITY_SCALE.find(item => item.speed === speed)?.label;
};

/**The speed the interface should initialize with */
export const DEFAULT_VELOCITY_SCALE: number = VELOCITY_SCALE[1].speed;

/**
 * Set of buttons so the user can control the scaling of the speed for all controls.
 * @param props see {@link SpeedControlProps}
 */
export const ActionSpeed = (props: ActionSpeedProps) => {
    const [isModalOpen, setIsModalOpen] = React.useState<boolean>(false);

    return (
        <div className="action-speed">
            <ModalActionSpeed isOpen={isModalOpen} handleClose={(newSpeedLabel: string) => {
                setIsModalOpen(false);
                props.setCameraVeilCallback(false);
                props.onChange(getSpeedByLabel(newSpeedLabel))
            }} />
            <button
                onClick={() => {
                    setIsModalOpen(!isModalOpen);
                    props.setCameraVeilCallback(!isModalOpen)
                    buttonFunctionProvider.disableActiveButton()
                }}
                aria-label="Change action speed"
                aria-hidden={props.isCameraVeilVisible}
            >
                <span className={`action-speed-icon ${getLabelBySpeed(props.speed)}`}></span>
            </button>
        </div>
    );
};

interface ModalActionSpeedProps {
    isOpen: boolean;
    /**
     * Function handles behavior modal close
     * @param newSpeedLabel the label for the newly selected speed
     */
    handleClose: (newSpeedLabel: string) => void;
}

interface OptionItem {
    value: string;
}

const ModalActionSpeed: React.FC<ModalActionSpeedProps> = ({ isOpen, handleClose }) => {
    const [selectedSpeed, setSelectedSpeed] = useState<string>(VELOCITY_SCALE[1].label);

    const options: OptionItem[] = VELOCITY_SCALE.map(item => ({
        value: item.label
    }));

    const handleSpeedSelection = (speed: string) => {
        setSelectedSpeed(speed);
        setTimeout(() => handleClose(speed), 500)
    };

    return (
        <Modal
            isOpen={isOpen}
            onClose={() => handleClose}
            title="Action Speed"
            subtitle="SELECT"
            modalClassName="action-speed-modal"
        >
            <div className="action-speed-options">
                {options.map(opt => {
                    const ariaLabel = `Select \"${opt.value}\" speed`;

                    return (
                        <button
                            key={opt.value}
                            className={`${opt.value} ${selectedSpeed === opt.value ? 'selected' : ''}`}
                            aria-label={ariaLabel}
                            aria-hidden={!isOpen}
                            onClick={() => handleSpeedSelection(opt.value)}
                        >
                            <span className="aria-inviz"></span>
                        </button>
                    )
                })}
            </div>
            <div>
                <div className="action-speed-labels">
                    {options.map(opt => (
                        <div
                            key={`label-${opt.value}`}
                            className={`speed-label ${selectedSpeed === opt.value ? 'selected' : ''}`}
                            aria-hidden="true"
                        >
                            {opt.value.charAt(0).toUpperCase() + opt.value.slice(1)}
                        </div>
                    ))}
                </div>
            </div>
        </Modal>
    );
};