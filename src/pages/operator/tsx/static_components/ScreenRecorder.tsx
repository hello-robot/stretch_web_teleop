import { createInstance, setup } from "@loomhq/record-sdk";
import { isSupported } from "@loomhq/record-sdk/is-supported";
import { oembed } from "@loomhq/loom-embed";
import { useEffect, useState } from "react";
import React from "react";

const PUBLIC_APP_ID = "3e587f93-4de1-43d8-aadc-2d023974eab5";
const BUTTON_ID = "loom-record-sdk-button";

export const ScreenRecorder = () => {
  const [videoHTML, setVideoHTML] = useState("");
  const [jws, setJws] = useState<string | null>(null);

//   useEffect(() => {
//     async function fetchJws() {
//       const response = await fetch("/api/jws");
//       const data = await response.json();
//       setJws(data.jws);
//     }

//     fetchJws();
    
//   }, []);

  useEffect(() => {
    async function setupLoom() {
        const { supported, error } = await isSupported();

        if (!supported) {
            console.warn(`Error setting up Loom: ${error}`);
            return;
        }

        const button = document.getElementById(BUTTON_ID);

        const { configureButton } = await createInstance({
            mode: "standard",
            publicAppId: PUBLIC_APP_ID,
        });

        console.log("configured")
        const sdkButton = configureButton({ element: button });
    }

    setupLoom();
  }, []);

  return (
        <>
            <button id={BUTTON_ID}>Record</button>
        </>
  );
}