import { addElements, getAssetBasePath } from '@frc-web-components/app';
import { getField3dDashboardConfigs, UrdfConfig } from '@frc-web-components/fwc/components/field3d';

const urdfConfigs: UrdfConfig[] = [
  {
    name: "2423-simple",
    position: [0, 0, 0],
    rotations: [{ axis: "z", degrees: -90 }],
    src: "/urdf-2423-simple/robot.urdf"
  },
];

addElements(getField3dDashboardConfigs({ assetPathPrefix: getAssetBasePath(), urdfConfigs }), 'FRC');
