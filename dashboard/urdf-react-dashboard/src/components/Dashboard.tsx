import {
  useEntry,
  Field3d,
  Field3dUrdf,
  useJson,
  useNt4,
  SendableChooser
} from "@frc-web-components/react";
import { UrdfConfig } from "@frc-web-components/fwc/components/field3d";
import { useEffect, useRef, useState } from "react";

const urdfConfigs: UrdfConfig[] = [
  {
    name: "2423-simple",
    position: [0, 0, 0],
    rotations: [{ axis: "z", degrees: -90 }],
    src: "/urdf-2423-simple/robot.urdf"
  },
];

const Dashboard = () => {
  const [, setJointNames] = useState<string[]>([]);
  const joints: Record<string, number> = useJson('/joints', {}, false);
  const { nt4Provider } = useNt4();
  const [pose3d] = useEntry('/a/b', [0, 0, 0]);
  const [robotPose] = useEntry('/SmartDashboard/Field/Robot', [0, 0, 0]);
  const [time] = useEntry('/Time', 0)
  const [isRedAlliance] = useEntry('/FMSInfo/IsRedAlliance', true)
  const origin = isRedAlliance ? 'red' : 'blue';

  const urdfRef = useRef<any>();

  const setJoint = (key: string, value: number) => {
    nt4Provider.setValue(`/joints/${key}`, value);
  }

  useEffect(() => {
    urdfRef.current?.setAngles(joints);
  }, [joints]);

  return (
    <div style={{
      display: 'flex',
      width: '100vw',
      height: '100vh'
    }}>
      <div style={{
        height: '100%',
        overflow: 'auto'
      }}>
        <div style={{ display: 'flex', gap: '10px', flexDirection: 'column', width: '300px' }}>
          <div>Time: {time.toFixed(2)}</div>
          <div>Alliance: {origin}</div>
          <SendableChooser source-key="/Shuffleboard/Autonomous/SendableChooser[0]" />
        </div>
      </div>
      <Field3d origin={origin} urdfConfigs={urdfConfigs} style={{
        flex: '1',
        height: '100%',
      }} cameraPose={pose3d} fixedCamera={false}>
        <Field3dUrdf name="2423-simple" pose={robotPose} onurdfload={((ev: any) => {
          const urdf = (ev as any).detail.urdf;
          urdfRef.current = urdf;
          const newJointValues: Record<string, number> = {};
          setJointNames(Object.keys(urdf.joints).sort());
          Object.entries(urdf.joints).forEach(([key, joint]) => {
            (joint as any).ignoreLimits = true;
            newJointValues[key] = 0;
            setJoint(key, 0);
          });
        })} />
      </Field3d>
    </div>
  );
};

export default Dashboard;
