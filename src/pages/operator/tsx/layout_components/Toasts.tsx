import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import '../../css/Toasts.css';

export interface Toast {
    id: string;
    type: 'success' | 'error' | 'info';
    message: string;
    duration?: number;
    closeButton?: boolean;
}

/**
 * Toast component that displays a notification message.
 * @param {ToastProps} props - The properties for the toast.
 * @returns {JSX.Element} The rendered toast component.
 */
const Toast: React.FC<Toast> = ({
    id,
    type = 'info',
    message,
    duration = 2000,
    closeButton = false
}) => {
    const [isVisible, setIsVisible] = useState<boolean>(true);

    useEffect(() => {
        // If duration is -1 don't
        // auto-dismiss the toast...
        if (duration === -1) return;
        // ..else setTimeout() to auto-dismiss toast
        if (duration > 0) {
            const timer = setTimeout(() => {
                setIsVisible(false);
            }, duration);

            return () => clearTimeout(timer);
        }
    }, [id, duration]);

    return (
        <AnimatePresence>
            {isVisible && (
                <motion.div
                    initial={{ opacity: 0, y: -50 }}
                    animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: -10 }}
                    transition={{ duration: 0.2 }}
                    layout
                    className={`toast-notification toast-${type}`}
                >
                    <span className="toast-message">{message}</span>
                    {closeButton && <button
                        onClick={() => setIsVisible(false)}
                        className="toast-close"
                    >
                        ✕
                    </button>}
                </motion.div>
            )}
        </AnimatePresence>
    );
};

interface ToastsProps {
    toasts: Toast[];
    toastsSet: React.Dispatch<React.SetStateAction<Toast[]>>;
}

/**
 * Toasts component that manages and displays a list of toast notifications.
 * @param {ToastsProps} props - The properties for the toasts component.
 * @returns {JSX.Element} The rendered toasts component.
 */
const Toasts: React.FC<ToastsProps> = ({ toasts, toastsSet }) => {
    const removeToast = (id: string) => {
        toastsSet((prevToasts) => prevToasts.filter((toast) => toast.id !== id));
    };

    return (
        <div className="toast-container">
            <AnimatePresence>
                {toasts.slice().reverse().map((toast) => (
                    <motion.div
                        key={toast.id}
                        onAnimationComplete={(definition) => {
                            removeToast(toast.id);
                        }}
                    >
                        <Toast
                            id={toast.id}
                            type={toast.type}
                            message={toast.message}
                            duration={toast.duration}
                        />
                    </motion.div>
                ))}
            </AnimatePresence>
        </div>
    );
};

export default Toasts;