import React from "react";

type AudioStreamProps = {};

export class AudioStream extends React.Component<AudioStreamProps> {
  outputAudioStream?: MediaStream;

  constructor(props: AudioStreamProps) {
    super(props);
    this.outputAudioStream = new MediaStream();
  }

  async start() {
    this.outputAudioStream = await navigator.mediaDevices.getUserMedia({
      audio: true,
      video: false,
    });
  }
}
