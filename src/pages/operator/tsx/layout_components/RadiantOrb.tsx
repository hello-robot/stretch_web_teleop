import React, { useEffect, useState } from "react";
import { motion } from "framer-motion";

import "../../css/RadiantOrb.css";

interface RadiantOrbProps {
    children?: React.ReactNode;
}


const RadiantOrb: React.FC<RadiantOrbProps> = ({ children }) => {
    const [className, setClassName] = useState("radiant-orb");

    useEffect(() => {
        const timer = setTimeout(() => {
            setClassName("radiant-orb anim-bootstrapped");
        }, 1000);

        return () => clearTimeout(timer);
    }, []);

    return (
        <div className={className}>
            {children}
            <div className="gripper" />
        </div>
    );
};

export default RadiantOrb;
