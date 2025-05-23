import React, { useState, useEffect, useCallback } from 'react'
import "operator/css/ModalActionMode.css"

interface ModalActionModeProps {
  isOpen: boolean;
  handleClose: () => void;
}

type ActionMode = 'press' | 'tap' | 'step';
type AnimationState = '' | 'enter' | 'exit';

const ModalActionMode: React.FC<ModalActionModeProps> = ({ isOpen, handleClose }) => {
  const [mode, setMode] = useState<ActionMode>('tap')
  const [visible, setVisible] = useState<boolean>(isOpen)
  const [animState, setAnimState] = useState<AnimationState>('')
  const [loading, setLoading] = useState<boolean>(false)

  // handle open/close anims
  useEffect(() => {
    if (isOpen) {
      setVisible(true)
      requestAnimationFrame(() => setAnimState('enter'))
    } else if (visible) {
      setAnimState('exit')
    }
  }, [isOpen, visible])

  // unmount after exit anim
  const onAnimEnd = useCallback((e: React.AnimationEvent<HTMLDivElement>) => {
    if ((e.target as HTMLElement).classList.contains('modal') && animState === 'exit') {
      setVisible(false)
      setAnimState('')
    }
  }, [animState])

  if (!visible) return null

  interface OptionItem {
    value: ActionMode;
    label: string;
    desc: string;
  }

  const options: OptionItem[] = [
    { value: 'press', label: 'Press-Hold', desc: 'Stretch moves while you press and hold a button and stops when you release.' },
    { value: 'tap', label: 'Tap Tap', desc: 'Stretch moves when you tap a button and stops when you tap again.' },
    { value: 'step', label: 'Step Action', desc: 'Stretch moves a fixed distance based on the selected speed with each click.' },
  ]

  const handleConfirm = (): void => {
    if (loading) return
    setLoading(true)
    // This is a synthetic delay
    setTimeout(() => {
      setLoading(false)
      handleClose()
    }, 1000)
  }

  return (
    <div
      className={`modal-overlay ${animState}`}
      onAnimationEnd={onAnimEnd}
      onClick={(e: React.MouseEvent<HTMLDivElement>) => {
        if (e.target === e.currentTarget) {
          handleClose();
        }
      }
      }>
      <div className={`modal ${animState}`} onAnimationEnd={onAnimEnd}>
        <div className="modal-header">
          <div className="modal-subtitle">SELECT</div>
          <h2 className="modal-title">Action Mode</h2>
        </div>
        <div className="modal-body">
          {options.map(opt => (
            <label key={opt.value} className="radio-group">
              <input
                type="radio"
                name="actionMode"
                value={opt.value}
                checked={mode === opt.value}
                onChange={() => setMode(opt.value)}
                disabled={loading}
              />
              <span className="radio-label">{opt.label}</span>
              {mode === opt.value && (
                <p className="radio-desc">{opt.desc}</p>
              )}
            </label>
          ))}
        </div>
        <div className="modal-footer">
          <button
            className="btn btn-primary"
            onClick={handleConfirm}
            disabled={loading}
          >
            {loading
              ? <span className="spinner" />
              : 'Confirm'}
          </button>
        </div>
      </div>
    </div>
  )
}

export default ModalActionMode
