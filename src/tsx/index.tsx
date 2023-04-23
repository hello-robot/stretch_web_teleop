import React from 'react';
import ReactDOM from 'react-dom';
import { createRoot } from 'react-dom/client';
import '../css/index.css'

import { VideoStreamGrid, OverheadComponent } from './videostreams';
import { ImageViewer, Encoding, TransportLayer } from 'rosreact';
import { Connect } from './robot'

// Connect to ROS
ReactDOM.render(
<Connect/>,
document.getElementById('jointStates')
);

// Stream video streams
ReactDOM.render(
    <VideoStreamGrid/>,
    document.getElementById('root')
);

// const element = document.getElementById('root');
// const root = createRoot(element);
// root.render(<VideoStreamGrid />);