import React from "react";
import { createRoot } from "react-dom/client";
import { BaseSignaling } from "shared/signaling/Signaling";
import { LocalSignaling } from "shared/signaling/LocalSignaling";

// Exposes these modules to Playwrist and the Dev Console
window.BaseSignaling = BaseSignaling;
window.LocalSignaling = LocalSignaling;

const container = document.getElementById("root");
const root = createRoot(container!);
root.render(
    <p>Loaded.</p>
);
