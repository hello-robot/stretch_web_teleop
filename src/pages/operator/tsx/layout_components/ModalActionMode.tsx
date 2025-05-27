import React, { useState } from 'react';
import Modal from '../basic_components/ModalMobile'; // Adjusted import path
import "operator/css/ModalActionMode.css";

interface ModalActionModeProps {
  isOpen: boolean;
  handleClose: () => void;
}

type ActionModeValue = 'press' | 'tap' | 'step';

interface OptionItem {
  value: ActionModeValue;
  label: string;
  desc: string;
}

const ModalActionMode: React.FC<ModalActionModeProps> = ({ isOpen, handleClose }) => {
  const [selectedMode, setSelectedMode] = useState<ActionModeValue>('press'); // Default mode
  const [loading, setLoading] = useState<boolean>(false);

  const options: OptionItem[] = [
    { value: 'press', label: 'Press-Hold', desc: 'Stretch moves while you press and hold a button and stops when you release.' },
    { value: 'tap', label: 'Tap Tap', desc: 'Stretch moves when you tap a button and stops when you tap again.' },
    { value: 'step', label: 'Step Action', desc: 'Stretch moves a fixed distance based on the selected speed with each click.' },
  ];

  const handleConfirm = (): void => {
    if (loading) return;
    setLoading(true);
    // This is a synthetic delay, replace with actual logic
    console.log("Selected Action Mode:", selectedMode);
    setTimeout(() => {
      setLoading(false);
      handleClose(); // Close the modal
      // Potentially call a prop to update the action mode in the parent component
      // e.g., props.onModeChange(selectedMode);
    }, 1000);
  };

  const modalFooterContent = (
    <button
      className="btn btn-primary" // Style from ModalActionMode.css
      onClick={handleConfirm}
      disabled={loading}
    >
      {loading ? <span className="spinner" /> : 'Confirm'}
    </button>
  );

  return (
    <Modal
      isOpen={isOpen}
      onClose={handleClose}
      title="Action Mode"
      subtitle="SELECT"
      footer={modalFooterContent}
      modalClassName="action-mode-specific-modal" // Optional: for very specific overrides
    >
      <div className="action-mode-options">
        {options.map(opt => (
          <label key={opt.value} className={`radio-group ${selectedMode === opt.value ? 'selected' : ''}`}>
            <input
              type="radio"
              name="actionMode"
              value={opt.value}
              checked={selectedMode === opt.value}
              onChange={() => setSelectedMode(opt.value)}
              disabled={loading}
            />
            <span className="radio-label">{opt.label}</span>
            {selectedMode === opt.value && (
              <p className="radio-desc">{opt.desc}</p>
            )}
          </label>
        ))}
      </div>
    </Modal>
  );
};

export default ModalActionMode;