export {}

import React from "react";
import { createRoot } from "react-dom/client";
import { LayoutArea } from "./layoutarea"

const container = document.getElementById('root');
const root = createRoot(container!);
root.render(<LayoutArea />)