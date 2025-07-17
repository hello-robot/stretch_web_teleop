import React, { useState, useEffect, useCallback, ReactNode } from 'react';
import "operator/css/ModalMobile.css"

export type AnimationState = '' | 'enter' | 'exit';

interface ModalMobileProps {
    isOpen: boolean;
    title: string;
    onClose?: () => void;
    subtitle?: string;
    children: ReactNode;
    footer?: ReactNode;
    modalClassName?: string;
    overlayClassName?: string;
    HeaderControls?: React.ReactNode; // Optional node for header controls
}

/**
 * ModalMobile is a reusable modal component
 * that can be used to display content in a dialog format.
 * It supports animations for entering and exiting,
 * and can be customized with titles, subtitles, and footers.
 * 
 * @param isOpen - Controls the visibility of the modal.
 * @param title - Title of the modal.
 * @param onClose - Callback when the modal is closed.
 * @param subtitle - Optional subtitle for the modal.
 * @param children - Content to be displayed inside the modal.
 * @param footer - Optional footer content for the modal.
 * @param modalClassName - Additional CSS class for the modal content.
 * @param overlayClassName - Additional CSS class for the modal overlay.
 */

const ModalMobile: React.FC<ModalMobileProps> = ({
    isOpen,
    title,
    onClose = () => { },
    subtitle,
    children,
    footer,
    modalClassName = '',
    overlayClassName = '',
    HeaderControls,
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

    const onAnimationEnd = useCallback((e: React.AnimationEvent<HTMLDialogElement | HTMLDivElement>) => {
        // Ensure the animation event is from the modal itself and not a child
        if (
            (e.target as HTMLElement).classList.contains('modal-content-wrapper') &&
            animState === 'exit'
        ) {
            setVisible(false);
            setAnimState('');
            // Call onClose here if the modal is fully closed and not just hidden by animation
            // However, onClose is typically tied to user action or isOpen prop change.
            // If onClose needs to be called after animation, it should be handled carefully.

            onClose();
        }
    }, [animState]);

    const handleOverlayClick = (e: React.MouseEvent<HTMLDialogElement>) => {
        if (e.target === e.currentTarget) {
            onClose();
        }
    };
    if (!visible) return null

    return (
        <dialog
            className={`modal-overlay ${animState} ${overlayClassName}`}
            onClick={handleOverlayClick}
            onAnimationEnd={animState === 'exit' ? onAnimationEnd : undefined}
            role="dialog"
            aria-modal="true"
            aria-hidden={!visible}
            tabIndex={0}
        >
            <div
                className={`modal-content-wrapper ${animState} ${modalClassName}`}
                onAnimationEnd={onAnimationEnd} // Listen to modal slide animation
            >
                {(title || subtitle) && (
                    <div className="modal-header">
                        <div>
                            {subtitle && <div className="modal-subtitle" aria-hidden="true">{subtitle}</div>}
                            {title && <h2 className="modal-title" aria-hidden="true">{title}</h2>}
                        </div>
                        <div>
                            {HeaderControls}
                        </div>
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
        </dialog>
    );
};

export default ModalMobile;