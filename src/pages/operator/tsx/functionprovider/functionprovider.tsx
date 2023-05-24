import { RemoteRobot } from "shared/remoterobot"
import { VelocityCommand } from 'shared/commands'
import { ActionMode } from "../staticcomponents/actionmodebutton"
import { DEFAULT_VELOCITY_SCALE } from "../staticcomponents/velocitycontrol"
import { ValidJoints } from "shared/util";

/**
 * Provides logic to connect the {@link RemoteRobot} and the components in the 
 * interface
 */
export abstract class FunctionProvider {
    protected static remoteRobot?: RemoteRobot;
    public static velocityScale: number;
    public static actionMode: ActionMode;
    public activeVelocityAction?: VelocityCommand;
    public velocityExecutionHeartbeat?: number;

    /**
     * Adds a remote robot instance to this function provider. This must be called
     * before any components of the interface will be able to execute functions
     * to change the state of the robot.
     * 
     * @param remoteRobot the remote robot instance to add
     */
    static addRemoteRobot(remoteRobot: RemoteRobot) {
        FunctionProvider.remoteRobot = remoteRobot;
    }

    /**
     * Sets the initial values for velocity scale and action mode.
     */
    static initialize() {
        this.velocityScale = DEFAULT_VELOCITY_SCALE;
        this.actionMode = ActionMode.StepActions;
    }

    public incrementalBaseDrive(linVel: number, angVel: number) {
        this.stopCurrentAction()
        this.activeVelocityAction = FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
    }

    public incrementalArmMovement(jointName: ValidJoints, increment: number) {
        this.stopCurrentAction()
        this.activeVelocityAction = FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
    }

    public continuousBaseDrive(linVel: number, angVel: number) {
        this.stopCurrentAction()
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.driveBase(linVel, angVel),
            this.velocityExecutionHeartbeat = window.setInterval(() => {
                this.activeVelocityAction =
                    FunctionProvider.remoteRobot?.driveBase(linVel, angVel)
            }, 150);
    }

    public continuousArmMovement(jointName: ValidJoints, increment: number) {
        this.stopCurrentAction()
        this.activeVelocityAction =
            FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        this.velocityExecutionHeartbeat = window.setInterval(() => {
            this.activeVelocityAction =
                FunctionProvider.remoteRobot?.incrementalMove(jointName, increment)
        }, 150);
    }

    public stopCurrentAction() {
        if (this.activeVelocityAction) {
            // No matter what region this is, stop the currently running action
            this.activeVelocityAction.stop()
            this.activeVelocityAction = undefined
            clearInterval(this.velocityExecutionHeartbeat)
            this.velocityExecutionHeartbeat = undefined
        }
    }
}
