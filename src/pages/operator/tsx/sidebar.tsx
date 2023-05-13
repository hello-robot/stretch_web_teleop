type SidebarProps = {
    hidden: boolean
}

/** Popup on the right side of the screen while in customization mode. */
export const Sidebar = (props: SidebarProps) => {
    return (
        <div id="sidebar" hidden={props.hidden}>
            sidebar
        </div>
    )
}
