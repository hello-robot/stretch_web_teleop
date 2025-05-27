import React, { useState, useEffect, useCallback, ReactNode } from 'react';
import "operator/css/ModalMobile.css"

export type AnimationState = '' | 'enter' | 'exit';

interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  title: string;
  subtitle?: string;
  children: ReactNode;
  footer?: ReactNode;
  modalClassName?: string;
  overlayClassName?: string;
}

const Modal: React.FC<ModalProps> = ({
  isOpen,
  onClose,
  title,
  subtitle,
  children,
  footer,
  modalClassName = '',
  overlayClassName = '',
}) => {
  const [visible, setVisible] = useState<boolean>(isOpen);
  const [animState, setAnimState] = useState<AnimationState>('');

  useEffect(() => {
    if (isOpen) {
      setVisible(true);
      requestAnimationFrame(() => setAnimState('enter'));
    } else if (visible) {
      setAnimState('exit');
    }
  }, [isOpen, visible]);

  const onAnimationEnd = useCallback((e: React.AnimationEvent<HTMLDivElement>) => {
    // Ensure the animation event is from the modal itself and not a child
    if ((e.target as HTMLElement).classList.contains('modal-content-wrapper') && animState === 'exit') {
      setVisible(false);
      setAnimState('');
      // Call onClose here if the modal is fully closed and not just hidden by animation
      // However, onClose is typically tied to user action or isOpen prop change.
      // If onClose needs to be called after animation, it should be handled carefully.
    }
  }, [animState]);

  const handleOverlayClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  if (!visible) return null;

  return (
    <div
      className={`modal-overlay ${animState} ${overlayClassName}`}
      onClick={handleOverlayClick}
      onAnimationEnd={animState === 'exit' ? onAnimationEnd : undefined} // Only listen to overlay fade-out if needed
    >
      <div
        className={`modal-content-wrapper ${animState} ${modalClassName}`}
        onAnimationEnd={onAnimationEnd} // Listen to modal slide animation
      >
        {(title || subtitle) && (
          <div className="modal-header">
            {subtitle && <div className="modal-subtitle">{subtitle}</div>}
            {title && <h2 className="modal-title">{title}</h2>}
          </div>
        )}
        <div className="modal-body">
          {children}
        </div>
        {footer && (
          <div className="modal-footer">
            {footer}
          </div>
        )}
      </div>
    </div>
  );
};

export default Modal;