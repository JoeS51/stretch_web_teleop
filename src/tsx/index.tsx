import React from 'react';
import ReactDOM from 'react-dom';
import '../css/index.css'

import { Robot } from './robot'
import { WebRTCConnection } from './webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage } from '../util/util'
import { VideoStreamComponent, VideoStream } from './videostreams';

export const robot = new Robot({})
export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);
export let allRemoteStreams = new Map()

robot.connect().then(() => {
    robot.subscribeToVideo({
        topicName: "/rotatedNavCamera/compressed",
        callback: navigationStream.updateImage.bind(navigationStream)
    })
    navigationStream.start()

    robot.subscribeToVideo({
        topicName: "/rotatedCamera/compressed",
        callback: realsenseStream.updateImage.bind(realsenseStream)
    })
    realsenseStream.start()

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/compressed",
        callback: gripperStream.updateImage.bind(gripperStream)
    })
    gripperStream.start()

    connection = new WebRTCConnection({ 
        peerName: 'ROBOT', 
        onConnectionStart: handleSessionStart,
        onTrackAdded: handleRemoteTrackAdded,
        onMessage: handleMessage
    });
    connection.connectToRobot('ROBOT')
})

function handleSessionStart() {
    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "navigation"))

    stream = realsenseStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "realsense"))

    stream = gripperStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "gripper"))
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message)
        return
    }
    console.log(message)
};

function handleRemoteTrackAdded(event: RTCTrackEvent) {
    console.log('Remote track added.');
    const track = event.track;
    const stream = event.streams[0];
    console.log('got track id=' + track.id, track);
    if (stream) {
        console.log('stream id=' + stream.id, stream);
    }
    console.log('OPERATOR: adding remote tracks');

    let streamName = this.connection.cameraInfo[stream.id]
    allRemoteStreams.set(streamName, { 'track': track, 'stream': stream });
}

ReactDOM.render(
    <VideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>,
    document.getElementById('root')
);

