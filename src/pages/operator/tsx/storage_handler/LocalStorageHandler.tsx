import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";
import { RobotPose } from "shared/util";

/** Uses browser local storage to store data. */
export class LocalStorageHandler extends StorageHandler {
    public static CURRENT_LAYOUT_KEY = "user_custom_layout";    
    public static LAYOUT_NAMES_KEY = "user_custom_layout_names";
    public static POSE_NAMES_KEY = "user_pose_names";

    constructor(onStorageHandlerReadyCallback: () => void) {
        super(onStorageHandlerReadyCallback);
        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.onReadyCallback();
        }, 0);
    }

    public loadCustomLayout(layoutName: string): LayoutDefinition {
        const storedJson = localStorage.getItem(layoutName);
        if (!storedJson) throw Error(`Could not load custom layout ${layoutName}`);
        return JSON.parse(storedJson);
    }

    public saveCustomLayout(layout: LayoutDefinition, layoutName: string): void {
        const layoutNames = this.getCustomLayoutNames();
        layoutNames.push(layoutName);
        localStorage.setItem(LocalStorageHandler.LAYOUT_NAMES_KEY, JSON.stringify(layoutNames));
        localStorage.setItem(layoutName, JSON.stringify(layout));
    }

    public saveCurrentLayout(layout: LayoutDefinition): void {
        localStorage.setItem(LocalStorageHandler.CURRENT_LAYOUT_KEY, JSON.stringify(layout));
    }

    public loadCurrentLayout(): LayoutDefinition | null {
        const storedJson = localStorage.getItem(LocalStorageHandler.CURRENT_LAYOUT_KEY);
        if (!storedJson) return null;
        return JSON.parse(storedJson);
    }

    public getCustomLayoutNames(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.LAYOUT_NAMES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson);
    }

    public savePose(poseName: string, jointState: RobotPose): void {
        const poseNames = this.getPoseNames();
        if (!poseNames.includes(poseName)) poseNames.push(poseName);
        localStorage.setItem(LocalStorageHandler.POSE_NAMES_KEY, JSON.stringify(poseNames));
        localStorage.setItem(poseName, JSON.stringify(jointState));
    }

    public deletePose(poseName: string): void {
        const poseNames = this.getPoseNames();
        if (!poseNames.includes(poseName)) return;
        localStorage.removeItem(poseName)
        const index = poseNames.indexOf(poseName)
        poseNames.splice(index, 1)
        localStorage.setItem(LocalStorageHandler.POSE_NAMES_KEY, JSON.stringify(poseNames));
    }

    public getPoseNames(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.POSE_NAMES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson)
    }

    public getPose(poseName: string): RobotPose {
        const storedJson = localStorage.getItem(poseName);
        if (!storedJson) throw Error(`Could not load pose ${poseName}`);
        return JSON.parse(storedJson);
    }
}