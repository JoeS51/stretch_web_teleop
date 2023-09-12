import React from 'react'
import { ROSJointState, ROSCompressedImage, ValidJoints, VideoProps, ROSOccupancyGrid, ROSPose, FollowJointTrajectoryActionResult, MarkerArray, MoveBaseActionResult, ArucoMarkersInfo, ArucoNavigationState, MoveBaseState, ArucoNavigationFeedback } from 'shared/util';
import ROSLIB, { Goal, Message, Ros, Topic } from "roslib";
import { JOINT_LIMITS, RobotPose, generateUUID, waitUntil } from 'shared/util';
import { response } from 'express';

export var robotMode: "navigation" | "position" = "position"
export var rosConnected = false;
export var ros: ROSLIB.Ros = new ROSLIB.Ros({
    url: 'wss://localhost:9090'
});

export class Robot extends React.Component {
    private jointState?: ROSJointState;
    private poseGoal?: ROSLIB.Goal;
    private poseGoalComplete?: boolean
    private moveBaseGoal?: ROSLIB.Goal;
    private navigateToArucoGoal?: ROSLIB.Goal;
    private trajectoryClient?: ROSLIB.ActionClient;
    private moveBaseClient?: ROSLIB.ActionClient;
    private navigateToArucoClient?: ROSLIB.ActionClient;
    private cmdVelTopic?: ROSLIB.Topic;
    private switchToNavigationService?: ROSLIB.Service;
    private switchToPositionService?: ROSLIB.Service;
    private setCameraPerspectiveService?: ROSLIB.Service;
    private setDepthSensingService?: ROSLIB.Service;
    private setArucoMarkersService?: ROSLIB.Service;
    private navigateToArucoService?: ROSLIB.Service;
    private arucoMarkerUpdateService?: ROSLIB.Service;
    private getRelativePoseService?: ROSLIB.Service
    private robotFrameTfClient?: ROSLIB.TFClient;
    private mapFrameTfClient?: ROSLIB.TFClient;
    private arucoMarkerInfoParam?: ROSLIB.Param;
    private linkGripperFingerLeftTF?: ROSLIB.Transform
    private linkHeadTiltTF?: ROSLIB.Transform
    private jointStateCallback: (jointState: ROSJointState) => void
    private occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void
    private moveBaseResultCallback: (goalState: MoveBaseState) => void
    private amclPoseCallback: (pose: ROSLIB.Transform) => void
    private markerArrayCallback: (markers: MarkerArray) => void
    private arucoNavigationStateCallback: (state: ArucoNavigationState) => void
    private relativePoseCallback: (pose: ROSLIB.Transform) => void
    private lookAtGripperInterval?: number // ReturnType<typeof setInterval>

    constructor(props: {
        jointStateCallback: (jointState: ROSJointState) => void,
        occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void,
        moveBaseResultCallback: (goalState: MoveBaseState) => void,
        amclPoseCallback: (pose: ROSLIB.Transform) => void,
        markerArrayCallback: (markers: MarkerArray) => void,
        arucoNavigationStateCallback: (state: ArucoNavigationState) => void,
        relativePoseCallback: (pose: ROSLIB.Transform) => void
    }) {
        super(props);
        this.jointStateCallback = props.jointStateCallback
        this.occupancyGridCallback = props.occupancyGridCallback
        this.moveBaseResultCallback = props.moveBaseResultCallback
        this.amclPoseCallback = props.amclPoseCallback
        this.markerArrayCallback = props.markerArrayCallback
        this.arucoNavigationStateCallback = props.arucoNavigationStateCallback
        this.relativePoseCallback = props.relativePoseCallback
    }
    
    async connect(): Promise<void> {
        // ros = new ROSLIB.Ros({
        //     url: 'wss://localhost:9090'
        // });
        return new Promise<void>((resolve, reject) => {
            ros.on('connection', async () => {
                await this.onConnect();
                resolve()
            })
            ros.on('error', (error) => {
                reject(error)
            });

            ros.on('close', () => {
                reject('Connection to websocket has been closed.')
            });
        });
    }

    async onConnect() {
        this.subscribeToJointState()
        this.subscribeToOccupancyGrid()
        this.subscribeToMarkerArray()
        // this.subscribeToArucoNavigationState()
        this.subscribeToJointTrajectoryResult()
        this.subscribeToMoveBaseResult()
        this.subscribeToNavigateToArucoFeedback()
        this.createTrajectoryClient()
        this.createMoveBaseClient()
        this.createNavigateToArucoClient()
        this.createCmdVelTopic()
        this.createSwitchToNavigationService()
        this.createSwitchToPositionService()
        this.createSetCameraPerspectiveService()
        this.createDepthSensingService()
        this.createArucoMarkerService()
        this.createArucoNavigationService()
        this.createArucoMarkerUpdateService()
        this.createGetRelativePoseService()
        this.createRobotFrameTFClient()
        this.createMapFrameTFClient()
        this.createArucoMarkerParamServer()
        this.subscribeToGripperFingerTF()
        this.subscribeToHeadTiltTF()
        this.subscribeToMapTF()

        return Promise.resolve()
    }
    
    subscribeToJointState() {
        const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: ros,
            name: '/stretch/joint_states/',
            messageType: 'sensor_msgs/JointState'
        });
    
        jointStateTopic.subscribe((msg: ROSJointState) => {
            this.jointState = msg
            if (this.jointStateCallback) this.jointStateCallback(msg)
        });
    };
    
    subscribeToVideo(props: VideoProps) {
        let topic: ROSLIB.Topic<ROSCompressedImage> = new ROSLIB.Topic({
            ros: ros,
            name: props.topicName,
            messageType: 'sensor_msgs/CompressedImage'
        });
        topic.subscribe(props.callback)
    }
    
    subscribeToOccupancyGrid() {
        let topic: ROSLIB.Topic<ROSOccupancyGrid> = new ROSLIB.Topic({
            ros: ros,
            name: 'map',
            messageType: 'nav_msgs/OccupancyGrid',
            compression: 'png'
        })

        topic.subscribe((msg: ROSOccupancyGrid) => {
            console.log('occupancy grid')
            if (this.occupancyGridCallback) this.occupancyGridCallback(msg)
        })
    }

    subscribeToMarkerArray() {
        let topic: ROSLIB.Topic<MarkerArray> = new ROSLIB.Topic({
            ros: ros,
            name: 'aruco/marker_array',
            messageType: 'visualization_msgs/MarkerArray'
        })

        topic.subscribe((msg: MarkerArray) => {
            if (this.markerArrayCallback) this.markerArrayCallback(msg)
        })
    }

    subscribeToJointTrajectoryResult() {
        let topic: ROSLIB.Topic<FollowJointTrajectoryActionResult> = new ROSLIB.Topic({
            ros: ros,
            name: 'stretch_controller/follow_joint_trajectory/result',
            messageType: 'control_msgs/FollowJointTrajectoryActionResult'
        });
    
        topic.subscribe((msg: FollowJointTrajectoryActionResult) => {
            this.poseGoalComplete = msg.status.status > 2 ? true : false
        });
    };

    subscribeToMoveBaseResult() {
        let topic: ROSLIB.Topic<FollowJointTrajectoryActionResult> = new ROSLIB.Topic({
            ros: ros,
            name: 'move_base/result',
            messageType: 'move_base_msgs/MoveBaseActionResult'
        });
    
        topic.subscribe((msg: MoveBaseActionResult) => {
            if (this.moveBaseResultCallback) {
                if (msg.status.status == 2) this.moveBaseResultCallback({state: "Navigation cancelled!", alertType: "error"})
                else if (msg.status.status == 3) this.moveBaseResultCallback({state: "Navigation succeeded!", alertType: "success"})
                else this.moveBaseResultCallback({state: "Navigation failed!", alertType: "error"})
            }
        });
    };

    // subscribeToArucoNavigationState() {
    //     let topic: ROSLIB.Topic<ArucoNavigationState> = new ROSLIB.Topic({
    //         ros: ros,
    //         name: '/navigate_to_aruco/state',
    //         messageType: 'stretch_teleop_interface/ArucoNavigationState'
    //     })

    //     topic.subscribe((msg: ArucoNavigationState) => {
    //         if (this.arucoNavigationStateCallback) this.arucoNavigationStateCallback(msg)
    //     })
    // }

    subscribeToNavigateToArucoFeedback() {
        let topic: ROSLIB.Topic<ArucoNavigationFeedback> = new ROSLIB.Topic({
            ros: ros,
            name: '/navigate_to_aruco/feedback',
            messageType: 'stretch_teleop_interface/NavigateToArucoFeedback'
        })

        topic.subscribe((msg: ArucoNavigationFeedback) => {
            if (this.arucoNavigationStateCallback) this.arucoNavigationStateCallback(msg.feedback)
        })
    }

    createTrajectoryClient() {
        this.trajectoryClient = new ROSLIB.ActionClient({
            ros: ros,
            serverName: '/stretch_controller/follow_joint_trajectory',
            actionName: 'control_msgs/FollowJointTrajectoryAction',
            timeout: 100
        });
    }
    
    createMoveBaseClient() {
        this.moveBaseClient = new ROSLIB.ActionClient({
            ros: ros,
            serverName: '/move_base',
            actionName: 'move_base_msgs/MoveBaseAction',
            timeout: 100
        });
    }

    createNavigateToArucoClient() {
        this.navigateToArucoClient = new ROSLIB.ActionClient({
            ros: ros,
            serverName: '/navigate_to_aruco',
            actionName: 'stretch_teleop_interface/NavigateToArucoAction',
            timeout: 100
        })
    }

    createCmdVelTopic() {
        this.cmdVelTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/stretch/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    }
    
    createSwitchToNavigationService() {
        this.switchToNavigationService = new ROSLIB.Service({
            ros: ros,
            name: '/switch_to_navigation_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    createSwitchToPositionService() {
        this.switchToPositionService = new ROSLIB.Service({
            ros: ros,
            name: '/switch_to_position_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    createSetCameraPerspectiveService() {
        this.setCameraPerspectiveService = new ROSLIB.Service({
            ros: ros,
            name: '/camera_perspective',
            serviceType: 'stretch_teleop_interface/CameraPerspective'
        })
    }

    createDepthSensingService() {
        this.setDepthSensingService = new ROSLIB.Service({
            ros: ros,
            name: '/depth_ar',
            serviceType: 'stretch_teleop_interface/DepthAR'
        })
    }

    createArucoMarkerService() {
        this.setArucoMarkersService = new ROSLIB.Service({
            ros: ros,
            name: '/aruco_markers',
            serviceType: 'stretch_teleop_interface/ArucoMarkers'
        })
    }

    createArucoNavigationService() {
        this.navigateToArucoService = new ROSLIB.Service({
            ros: ros,
            name: '/navigate_to_aruco',
            serviceType: 'stretch_teleop_interface/NavigateToAruco'
        })
    }

    createArucoMarkerUpdateService() {
        this.arucoMarkerUpdateService = new ROSLIB.Service({
            ros: ros,
            name: '/aruco_marker_update',
            serviceType: 'stretch_teleop_interface/ArucoMarkerInfoUpdate'
        })
    }

    createGetRelativePoseService() {
        this.getRelativePoseService = new ROSLIB.Service({
            ros: ros,
            name: '/get_relative_pose',
            serviceType: 'stretch_teleop_interface/RelativePose'
        })
    }

    createRobotFrameTFClient() {
        this.robotFrameTfClient = new ROSLIB.TFClient({
            ros: ros,
            fixedFrame: 'base_link',
            angularThres: 0.001,
            transThres: 0.001,
            rate: 10
        });
    }

    createMapFrameTFClient() {
        this.mapFrameTfClient = new ROSLIB.TFClient({
            ros: ros,
            fixedFrame: 'map',
            angularThres: 0.001,
            transThres: 0.001,
            rate: 10
        });
    }

    createArucoMarkerParamServer() {
        this.arucoMarkerInfoParam = new ROSLIB.Param({
            ros : ros,
            name : 'aruco_marker_info'
        });
    }

    subscribeToGripperFingerTF() {
        this.robotFrameTfClient?.subscribe('link_gripper_finger_left', transform => {
            this.linkGripperFingerLeftTF = transform;
        });
    }

    subscribeToHeadTiltTF () {
        this.robotFrameTfClient?.subscribe('link_head_tilt', transform => {
            this.linkHeadTiltTF = transform;
        })
    }

    subscribeToMapTF() {
        this.mapFrameTfClient?.subscribe('base_link', transform => {
            if (this.amclPoseCallback) this.amclPoseCallback(transform)
        })
    }

    setCameraPerspective(props: {camera: "overhead" | "realsense" | "gripper", perspective: string}) {
        var request = new ROSLIB.ServiceRequest({camera: props.camera, perspective: props.perspective})
        this.setCameraPerspectiveService?.callService(request, (response: boolean) => {
            response ? console.log("Set " + props.camera + " to " + props.perspective + " perspective") 
                     : console.error(props.perspective + " is not a valid perspective for " + props.camera + " camera!")
        })
    }

    setDepthSensing(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({enable: toggle})
        this.setDepthSensingService?.callService(request, (response: boolean) => {
            response ? console.log("Enable depth sensing") : console.log("Disabled depth sensing")
        })
    }

    setArucoMarkers(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({enable: toggle})
        this.setArucoMarkersService?.callService(request, (response: boolean) => {
            response ? console.log("Enable aruco markers") : console.log("Disabled aruco markers")
        })
    }

    setArucoMarkerInfo(info: ArucoMarkersInfo) {
        console.log(info.toString())
        this.arucoMarkerInfoParam?.set(JSON.stringify(info), (response) => {
            console.log(response);
        })
    }

    navigateToArucoMarkers(marker_name: string, relative_pose: ROSLIB.Transform) {
        var request = new ROSLIB.ServiceRequest({
            name: marker_name, 
            pose: { 
                translation: relative_pose.translation,
                rotation: relative_pose.rotation
            }
        })
        console.log(request)
        this.navigateToArucoService?.callService(request, (response: string) => {
            console.log(response)
        })
    }

    updateArucoMarkersInfo() {
        var request = new ROSLIB.ServiceRequest({update: true})
        this.arucoMarkerUpdateService?.callService(request, (response: boolean) => {
            response ? console.log("Aruco marker dictionary updated") : console.log("Aruco marker dictionary update failed!") 
        })
    }

    getRelativePose(marker_name: string) {
        var request = new ROSLIB.ServiceRequest({name: marker_name})
        this.getRelativePoseService?.callService(request, (response: ROSLIB.Transform) => {
            this.relativePoseCallback(response) 
        })
    }

    switchToNavigationMode() {
        var request = new ROSLIB.ServiceRequest({});
        this.switchToNavigationService!.callService(request, () => {
            robotMode = "navigation"
            console.log("Switched to navigation mode")
        });
    }
    
    switchToPositionMode = () => {
        var request = new ROSLIB.ServiceRequest({});
        this.switchToPositionService!.callService(request, () => {
            robotMode = "position"
            console.log("Switched to position mode")
        });
    }

    executeBaseVelocity = (props: {linVel: number, angVel: number}): void => {
        this.stopExecution()
        let twist = new ROSLIB.Message({
            linear: {
                x: props.linVel,
                y: 0,
                z: 0
            },
            angular: {
                x: 0,
                y: 0,
                z: props.angVel
            }
        });
        if (!this.cmdVelTopic) throw 'trajectoryClient is undefined';
        this.cmdVelTopic.publish(twist)
    }

    makeIncrementalMoveGoal(jointName: ValidJoints, jointValueInc: number): ROSLIB.Goal | undefined {
        if (!this.jointState) throw 'jointState is undefined';
        let newJointValue = GetJointValue({ jointStateMessage: this.jointState, jointName: jointName })
        // Paper over Hello's fake joints
        if (jointName === "translate_mobile_base" || jointName === "rotate_mobile_base") {
            // These imaginary joints are floating, always have 0 as their reference
            newJointValue = 0
        } 

        let collision = inCollision({ jointStateMessage: this.jointState, jointName: jointName })
        let collisionIndex = jointValueInc <= 0 ? 0 : 1
        if (jointName === "joint_wrist_yaw") {
            collisionIndex = jointValueInc <= 0 ? 1 : 0
        }
        // Negative joint increment is for lower/retract/wrist out
        // Positive joint increment is for lift/extend/wrist in
        let index = jointValueInc <= 0 ? 0 : 1 
        // If request to move the joint in the direction of collision, cancel movement
        if (collision[collisionIndex]) return;

        newJointValue = newJointValue + jointValueInc

        // Make sure new joint value is within limits
        if (jointName in JOINT_LIMITS) {
            let inLimits = inJointLimitsHelper({ jointValue: newJointValue, jointName: jointName })
            if (!inLimits) throw 'invalid joint name'
            console.log(newJointValue, JOINT_LIMITS[jointName]![index], inLimits[index])
            if (!inLimits[index]) newJointValue = JOINT_LIMITS[jointName]![index]
        }

        let pose = { [jointName]: newJointValue }
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        return this.makePoseGoal(pose)
    }

    makeNavigateToArucoGoal(name: string, pose: ROSLIB.Transform) {
        if (!this.navigateToArucoClient) throw 'navigateToArucoClient is undefined';

        let newGoal = new ROSLIB.Goal({
            actionClient: this.navigateToArucoClient,
            goalMessage: {
                name: name,
                pose: pose
            }
        })

        return newGoal
    }

    makeMoveBaseGoal(pose: ROSPose) {
        if (!this.moveBaseClient) throw 'moveBaseClient is undefined';

        let newGoal = new ROSLIB.Goal({
            actionClient: this.moveBaseClient,
            goalMessage: {
                target_pose: {
                    header: {
                        frame_id: 'map'
                    },
                    pose: pose
                }
            }
        })

        // newGoal.on('result', (result) => {
        //     this.moveBaseResultCallback(result)
        // });
        
        return newGoal
    }

    makePoseGoal(pose: RobotPose) {
        let jointNames: ValidJoints[] = []
        let jointPositions: number[] = []
        for (let key in pose) {
            jointNames.push(key as ValidJoints)
            jointPositions.push(pose[key as ValidJoints]!)
        }
    
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        let newGoal = new ROSLIB.Goal({
            actionClient: this.trajectoryClient,
            goalMessage: {
                trajectory: {
                    header: {
                        stamp: {
                            secs: 0,
                            nsecs: 0
                        }
                    },
                    joint_names: jointNames,
                    points: [
                        {
                            positions: jointPositions,
                            // The following might causing the jumpiness in continuous motions
                            time_from_start: {
                                secs: 1,
                                nsecs: 0
                            }
    
                        }
                    ]
                }
            }
        });
    
        // newGoal.on('result', function (result) {
        //     console.log('Final Result: ' + result);
        // });

        return newGoal
    }

    makePoseGoals(poses: RobotPose[]) {
        let jointNames: ValidJoints[] = []
        for (let key in poses[0]) {
            jointNames.push(key as ValidJoints)
        }

        let points: any = []
        let jointPositions: number[] = []
        poses.forEach((pose, index) => {
            jointPositions = []
            for (let key in pose) {
                jointPositions.push(pose[key as ValidJoints]!)
            }
            points.push({
                positions: jointPositions,
                time_from_start: {
                    secs: 10,
                    nsecs: 0
                }
            })
        });

        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        let newGoal = new ROSLIB.Goal({
            actionClient: this.trajectoryClient,
            goalMessage: {
                trajectory: {
                    header: {
                        stamp: {
                            secs: 0,
                            nsecs: 0
                        }
                    },
                    joint_names: jointNames,
                    points: points
                },
            }
        });

        return newGoal
    }

    executePoseGoal(pose: RobotPose) {
        this.stopExecution();
        this.trajectoryClient?.cancel();
        this.poseGoal = this.makePoseGoal(pose)
        this.poseGoal.send()
    }

    async executePoseGoals(poses: RobotPose[], index: number) {
        this.stopExecution();
        this.poseGoal = this.makePoseGoals(poses)
        this.poseGoal.send()
    }

    executeMoveBaseGoal(pose: ROSPose) {
        this.stopExecution()
        this.moveBaseGoal = this.makeMoveBaseGoal(pose)
        // this.moveBaseResultCallback({state: "Navigating to selected goal...", alertType: "info"})
        this.moveBaseGoal.send()
    }

    executeNavigateToArucoGoal(name: string, pose: ROSLIB.Transform) {
        this.stopExecution()
        this.navigateToArucoGoal = this.makeNavigateToArucoGoal(name, pose)
        this.navigateToArucoGoal.send()
    }

    executeIncrementalMove(jointName: ValidJoints, increment: number) {
        this.stopExecution();
        this.poseGoal = this.makeIncrementalMoveGoal(jointName, increment)
        if (!this.poseGoal) {
            console.log("Joint in collision!")
            return;
        }
        this.poseGoal.send()
    }

    stopExecution() {
        this.stopTrajectoryClient()
        this.stopMoveBaseClient()    
        this.stopNavigateToArucoClient()
    }

    stopTrajectoryClient() {
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        this.trajectoryClient.cancel()
        if (this.poseGoal) {
            this.poseGoal.cancel()
            this.poseGoal = undefined
        }
    }

    stopMoveBaseClient() {
        if (!this.moveBaseClient) throw 'moveBaseClient is undefined';
        this.moveBaseClient.cancel()
        if (this.moveBaseGoal) {
            this.moveBaseGoal.cancel()
            this.moveBaseGoal = undefined
        }
    }

    stopNavigateToArucoClient() {
        if (!this.navigateToArucoClient) throw 'navigateToArucoClient is undefined';
        this.navigateToArucoClient.cancel()
        if (this.navigateToArucoGoal) {
            this.navigateToArucoGoal.cancel()
            this.navigateToArucoGoal = undefined
        }
    }

    setPanTiltFollowGripper(followGripper: boolean) {
        if (this.lookAtGripperInterval && followGripper) return;
        
        if (followGripper) { 
            let panOffset = 0;
            let tiltOffset = 0;
            let lookIfReadyAndRepeat = () => {
                if (this.linkGripperFingerLeftTF && this.linkHeadTiltTF) {
                    this.lookAtGripper(panOffset, tiltOffset);
                }
                this.lookAtGripperInterval = window.setTimeout(lookIfReadyAndRepeat, 500)
            }
            lookIfReadyAndRepeat()

            // this.lookAtGripperInterval = window.setTimeout(() => { 
            //     if (this.linkGripperFingerLeftTF && this.linkHeadTiltTF) {
            //         this.lookAtGripper(panOffset, tiltOffset);
            //     }
            // }, 500)
        } else {
            this.stopExecution()
            clearTimeout(this.lookAtGripperInterval)
            this.lookAtGripperInterval = undefined
        }
    }

    lookAtGripper(panOffset: number, tiltOffset: number) {
        if (!this.linkGripperFingerLeftTF) throw 'linkGripperFingerLeftTF is undefined';
        if (!this.linkHeadTiltTF) throw 'linkHeadTiltTF is undefined';
        let posDifference = {
            x: this.linkGripperFingerLeftTF.translation.x - this.linkHeadTiltTF.translation.x,
            y: this.linkGripperFingerLeftTF.translation.y - this.linkHeadTiltTF.translation.y,
            z: this.linkGripperFingerLeftTF.translation.z - this.linkHeadTiltTF.translation.z
        };

        // Normalize posDifference
        const scalar = Math.sqrt(posDifference.x ** 2 + posDifference.y ** 2 + posDifference.z ** 2);
        posDifference.x /= scalar;
        posDifference.y /= scalar;
        posDifference.z /= scalar;

        const pan = Math.atan2(posDifference.y, posDifference.x) + panOffset;
        const tilt = Math.atan2(posDifference.z, -posDifference.y) + tiltOffset;
        
        // Goals really close to current state cause some whiplash in these joints in simulation. 
        // Ignoring small goals is a temporary fix
        if (!this.jointState) throw 'jointState is undefined';
        let panDiff = Math.abs(GetJointValue({
            jointStateMessage: this.jointState, 
            jointName: "joint_head_pan"
        }) - pan);
        let tiltDiff = Math.abs(GetJointValue({
            jointStateMessage: this.jointState, 
            jointName: "joint_head_tilt"
        }) - tilt);
        if (panDiff < 0.02 && tiltDiff < 0.02) {
            return
        }

        this.executePoseGoal({
            'joint_head_pan': pan + panOffset,
            'joint_head_tilt': tilt + tiltOffset
        })
    }
}

export const GetJointValue = (props: { jointStateMessage: ROSJointState, jointName: ValidJoints }): number => {
    // Paper over Hello's fake joint implementation
    if (props.jointName === "wrist_extension") {
        return GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l0"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l1"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l2"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l3"});
    } else if (props.jointName === "translate_mobile_base" || props.jointName === "rotate_mobile_base") {
        return 0;
    }

    let jointIndex = props.jointStateMessage.name.indexOf(props.jointName)
    return props.jointStateMessage.position[jointIndex]
}

export function inJointLimits(props: { jointStateMessage: ROSJointState, jointName: ValidJoints }) {
    let jointValue = GetJointValue(props)
    return inJointLimitsHelper({ jointValue: jointValue, jointName: props.jointName})
}

function inJointLimitsHelper(props: { jointValue: number, jointName: ValidJoints }) {
    let jointLimits = JOINT_LIMITS[props.jointName]
    if (!jointLimits) return;

    var eps = 0.03
    let inLimits: [boolean, boolean] = [true, true]
    inLimits[0] = props.jointValue - eps >= jointLimits[0] // Lower joint limit
    inLimits[1] = props.jointValue + eps <= jointLimits[1] // Upper joint limit
    return inLimits
}

export function inCollision(props: { jointStateMessage: ROSJointState, jointName: ValidJoints }) {
    let inCollision: [boolean, boolean] = [false, false]
    const MAX_EFFORTS: { [key in ValidJoints]?: [number, number] } = {
        "joint_head_tilt": [-50, 50],
        "joint_head_pan": [-50, 50],
        "wrist_extension": [-40, 40],
        "joint_lift": [0, 70],
        "joint_wrist_yaw": [-10, 10],
    }

    if (!(props.jointName in MAX_EFFORTS)) return inCollision;

    let jointIndex = props.jointStateMessage.name.indexOf(props.jointName)
    // In collision if joint is applying more than 50% effort when moving downward/inward/backward
    inCollision[0] = props.jointStateMessage.effort[jointIndex] < MAX_EFFORTS[props.jointName]![0]
    // In collision if joint is applying more than 50% effort when moving upward/outward/forward
    inCollision[1] = props.jointStateMessage.effort[jointIndex] > MAX_EFFORTS[props.jointName]![1]

    return inCollision
}