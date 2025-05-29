import React, { useState } from 'react';
import Modal from '../basic_components/ModalMobile';
import "operator/css/ModalActionSpeed.css";

interface ModalActionSpeedProps {
  isOpen: boolean;
  handleClose: () => void;
}

interface OptionItem {
  value: string;
}

const ModalActionSpeed: React.FC<ModalActionSpeedProps> = ({ isOpen, handleClose }) => {
  const [selectedSpeed, setSelectedSpeed] = useState<string>('medium');

  const options: OptionItem[] = [{
    value: 'slow'
  }, {
    value: 'medium'
  }, {
    value: 'fast'
  }];

  const handleSpeedSelection = (speed: string) => {
    setSelectedSpeed(speed);
    setTimeout(handleClose, 500)
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={handleClose}
      title="Action Speed"
      subtitle="SELECT"
      modalClassName="action-speed-modal"
    >
      <div className="action-speed-options">
        {options.map(opt => {
          return (
            <button
              key={opt.value}
              className={`${opt.value} ${selectedSpeed === opt.value ? 'selected' : ''}`}
              aria-label={`Select ${opt.value} speed`}
              aria-hidden={!isOpen}
              onClick={() => handleSpeedSelection(opt.value)}
            >
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
            >
              {opt.value.charAt(0).toUpperCase() + opt.value.slice(1)}
            </div>
          ))}
        </div>
      </div>
    </Modal>
  );
};

export default ModalActionSpeed;