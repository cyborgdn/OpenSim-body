%% MATLAB script for the generation of a model
clc;
clear all;
close all;

% importing the existing model
import org.opensim.modeling.*
myModel = Model();

%% Geometry path
path = 'C:\OpenSim 4.1\Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);

%% Adding ground
ground = myModel.getGround();

% Add display geometry to the ground to visualize in the GUI
% For the ground plane, an arbitrary plane of 5X5X(0.2)mm^3 shape is taken
ground.attachGeometry(Mesh('basemodelv04.obj'));
% --------------------------------------------------------------------
%% Adding pelvis
pelvis = Body();
pelvis.setName('pelvis');
pelvis.setMass(11.777);
pelvis.setMassCenter(Vec3(-0.0707,0,0));
pelvis.setInertia(Inertia(0.1028,0.0871,0.0579,0,0,0));

% Adding the Joint
LocationInParent = Vec3(0,0,2);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

ground_pelvis = FreeJoint('ground_pelvis', ground, ...
    LocationInParent, OrientationInParent, pelvis, ...
    LocationInChild, OrientationInChild);

pelvis_tilt = ground_pelvis.upd_coordinates(0);
pelvis_tilt.setName('pelvis_tilt');
pelvis_tilt.setRange([-1.57079633, 1.57079633]);
pelvis_tilt.setDefaultValue(0);

pelvis_list = ground_pelvis.upd_coordinates(1);
pelvis_list.setName('pelvis_list');
pelvis_list.setRange([-1.57079633, 1.57079633]);
pelvis_list.setDefaultValue(0);

pelvis_rotation = ground_pelvis.upd_coordinates(2);
pelvis_rotation.setName('pelvis_rotation');
pelvis_rotation.setRange([-1.57079633, 1.57079633]);
pelvis_rotation.setDefaultValue(0);

pelvis_tx = ground_pelvis.upd_coordinates(3);
pelvis_tx.setRange([-5,5]);
pelvis_tx.setName('pelvis_tx');
pelvis_tx.setDefaultValue(0);
pelvis_tx.setDefaultSpeedValue(0);

pelvis_ty = ground_pelvis.upd_coordinates(4);
pelvis_ty.setRange([-1,2]);
pelvis_ty.setName('pelvis_ty');
pelvis_ty.setDefaultValue(0.95);
pelvis_ty.setDefaultSpeedValue(0);

pelvis_tz = ground_pelvis.upd_coordinates(5);
pelvis_tz.setRange([-3,3]);
pelvis_tz.setName('pelvis_tz');
pelvis_tz.setDefaultValue(0);
pelvis_tz.setDefaultSpeedValue(0);

% Add visible objects
pelvis.attachGeometry(Mesh('pelvis.vtp'));
pelvis.attachGeometry(Mesh('sacrum.vtp'));
pelvis.attachGeometry(Mesh('l_pelvis.vtp'));
myModel.addJoint(ground_pelvis);
% myModel.addJoint(ground_pelvis);
% pelvis.getDisplayer().setScaleFactors([1,1,1]);

% Add Body to Model
myModel.addBody(pelvis);
% ---------------------------------------------------------------------
%% Adding Femur_Right
femur_r = Body();
femur_r.setName('femur_r');
femur_r.setMass(9.3014);
femur_r.setMassCenter(Vec3(0,-0.17,0));
femur_r.setInertia(Inertia(0.1339,0.0351,0.1412,0,0,0));

% Adding the Joint
LocationInParent = Vec3(-0.0707,-0.0661,0.0835);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

hip_r = FreeJoint('hip_r', pelvis, ...
    LocationInParent, OrientationInParent, femur_r, ...
    LocationInChild, OrientationInChild);

hip_flexion_r = hip_r.upd_coordinates(0); % T about x
hip_flexion_r.setRange([-2.0943951, 2.0943951]);
hip_flexion_r.setName('hip_flexion_r');
hip_flexion_r.setDefaultValue(0);
hip_flexion_r.setDefaultSpeedValue(0);

hip_adduction_r = hip_r.upd_coordinates(1);
hip_adduction_r.setRange([-2.0943951, 2.0943951]);
hip_adduction_r.setName('hip_adduction_r');
hip_adduction_r.setDefaultValue(0);
hip_adduction_r.setDefaultSpeedValue(0);

hip_rotation_r = hip_r.upd_coordinates(1);
hip_rotation_r.setRange([-2.0943951, 2.0943951]);
hip_rotation_r.setName('hip_adduction_r');
hip_rotation_r.setDefaultValue(0);
hip_rotation_r.setDefaultSpeedValue(0);

femur_r.attachGeometry(Mesh('femur_r.vtp'));
myModel.addJoint(hip_r);

myModel.addBody(femur_r);
% --------------------------------------------------------------------
%% Adding Tibia_Right
tibia_r = Body();
tibia_r.setName('tibia_r');
tibia_r.setMass(3.7075);
tibia_r.setMassCenter(Vec3(0,-0.1867,0));
tibia_r.setInertia(Inertia(0.0504,0.0051,0.0511,0,0,0));

LocationInParent = Vec3(0,-0.38,0);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

knee_r = FreeJoint('knee_r', femur_r, ...
    LocationInParent, OrientationInParent, tibia_r, ...
    LocationInChild, OrientationInChild);

knee_angle_r = knee_r.upd_coordinates(0);
knee_angle_r.setName('knee_angle_r');
knee_angle_r.setRange([-2.0943951, 0.17453293]);
knee_angle_r.setDefaultValue(0);
knee_angle_r.setDefaultSpeedValue(0);

tibia_r.attachGeometry(Mesh('tibia_r.vtp'));
tibia_r.attachGeometry(Mesh('fibula.vtp'));
myModel.addJoint(knee_r);

myModel.addBody(tibia_r);
% --------------------------------------------------------------------
%% Adding the Talus Right
talus_r = Body();
talus_r.setName('talus_right');
talus_r.setMass(0.10000000000000001);
talus_r.setMassCenter(Vec3(0,-0.1867,0));
talus_r.setInertia(Inertia(0.0504,0.0051000000000000004,0.0511,0,0,0));

LocationInParent = Vec3(0,-0.43,0);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

ankle_r = FreeJoint('ankle_r', tibia_r, ...
    LocationInParent, OrientationInParent, talus_r, ...
    LocationInChild, OrientationInChild);

ankle_angle_r = ankle_r.upd_coordinates(0);
ankle_angle_r.setName('ankle_angle_r');
ankle_angle_r.setRange([-1.57079633, 1.57079633]);
ankle_angle_r.setDefaultValue(0);
ankle_angle_r.setDefaultSpeedValue(0);

talus_r.attachGeometry(Mesh('talus.vtp'));
myModel.addJoint(ankle_r);

myModel.addBody(talus_r);
% --------------------------------------------------------------------
%% Adding Calcn Right
calcn_r = Body();
calcn_r.setName('calcn_r');
calcn_r.setMass(1.25);
calcn_r.setMassCenter(Vec3(0.10000000000000001,0.02999999999999999,0));
calcn_r.setInertia(Inertia(0.0014,0.0038999999999999998,0.0041000000000000003,0,0,0));

LocationInParent = Vec3(-0.04877,-0.04195,0.00792);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

subtalar_r = FreeJoint('subtalar_r', talus_r, ...
    LocationInParent, OrientationInParent, calcn_r, ...
    LocationInChild, OrientationInChild);

subtalar_angle_r = subtalar_r.upd_coordinates(0);
subtalar_angle_r.setName('subtalar_angle_r');
subtalar_angle_r.setRange([-1.57079633, 1.57079633]);
subtalar_angle_r.setDefaultValue(0);
subtalar_angle_r.setDefaultSpeedValue(0);

calcn_r.attachGeometry(Mesh('foot.vtp'));
myModel.addJoint(subtalar_r);

myModel.addBody(calcn_r);
% --------------------------------------------------------------------
%% Adding Toes_right
toes_r = Body();
toes_r.setName('toes_r');
toes_r.setMass(0.21659999999999999);
toes_r.setMassCenter(Vec3(0.034599999999999999,0.0060000000000000001,-0.017500000000000002));
toes_r.setInertia(Inertia(0.0001,0.00020000000000000001,0.0001,0,0,0));

LocationInParent = Vec3(0.1788,-0.002,0.00108);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

mtp_r = FreeJoint('mtp_r', calcn_r, ...
    LocationInParent, OrientationInParent, ...
    toes_r, LocationInChild, OrientationInChild);

mtp_angle_r = mtp_r.upd_coordinates(0);
mtp_angle_r.setName('mtp_angle_r');
mtp_angle_r.setRange([-1.57079633, 1.57079633]);
mtp_angle_r.setDefaultValue(0);
mtp_angle_r.setDefaultSpeedValue(0);

toes_r.attachGeometry(Mesh('bofoot.vtp'));
myModel.addJoint(mtp_r);

myModel.addBody(toes_r);
% --------------------------------------------------------------------
%% Adding the prosthesis
% pros_r = Body();
% pros_r.setName('pros_r');
% pros_r.setMass(1.5666);
% pros_r.setMassCenter(Vec3(0,-0.1867,0));
% pros_r.setInertia(Inertia(0,0,0,0,0,0));
% 
% LocationInParent = Vec3(0,0,0);
% OrientationInParent = Vec3(0,0,0);
% LocationInChild = Vec3(0,0,0);
% OrientationInChild = Vec3(0,-90,0);
% 
% foot_r = FreeJoint('knee_r', tibia_r, ...
%     LocationInParent, OrientationInParent, pros_r, ...
%     LocationInChild, OrientationInChild);
% 
% foot_angle_l = foot_r.upd_coordinates(0);
% foot_angle_l.setName('foot_angle_r');
% foot_angle_l.setRange([-2.0943951, 0.17453293]);
% foot_angle_l.setDefaultValue(0);
% foot_angle_l.setDefaultSpeedValue(0);
% 
% pros_r.attachGeometry(Mesh('humanfoot.STL'));
% myModel.addJoint(foot_r);
% 
% myModel.addBody(pros_r);

% --------------------------------------------------------------------
%% Adding Femur_Left
femur_l = Body();
femur_l.setName('femur_l');
femur_l.setMass(9.3014);
femur_l.setMassCenter(Vec3(0,-0.17,0));
femur_l.setInertia(Inertia(0.1339,0.0351,0.14121,0,0,0));

LocationInParent = Vec3(-0.0707,-0.0611,-0.0835);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

hip_l = FreeJoint('hip_l', pelvis, ...
    LocationInParent, OrientationInParent, femur_l, ...
    LocationInChild, OrientationInChild);

hip_flexion_l = hip_l.upd_coordinates(0); % T about x
hip_flexion_l.setRange([-2.0943951, 2.0943951]);
hip_flexion_l.setName('hip_flexion_l');
hip_flexion_l.setDefaultValue(0);
hip_flexion_l.setDefaultSpeedValue(0);

hip_adduction_l = hip_l.upd_coordinates(1);
hip_adduction_l.setRange([-2.0943951, 2.0943951]);
hip_adduction_l.setName('hip_adduction_l');
hip_adduction_l.setDefaultValue(0);
hip_adduction_l.setDefaultSpeedValue(0);

hip_rotation_l = hip_l.upd_coordinates(2);
hip_rotation_l.setRange([-2.0943951, 2.0943951]);
hip_rotation_l.setName('hip_adduction_l');
hip_rotation_l.setDefaultValue(0);
hip_rotation_l.setDefaultSpeedValue(0);

femur_l.attachGeometry(Mesh('femur_l.vtp'));
myModel.addJoint(hip_l);

myModel.addBody(femur_l);
% --------------------------------------------------------------------
%% Adding Tibia_left
tibia_l = Body();
tibia_l.setName('tibia_l');
tibia_l.setMass(0.0504);
tibia_l.setMassCenter(Vec3(0,-0.1867,0));
tibia_l.setInertia(Inertia(0.0504,0.0051,0.0511,0,0,0));

LocationInParent = Vec3(0,-0.38,0);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

knee_l = FreeJoint('knee_r', femur_l, ...
    LocationInParent, OrientationInParent, tibia_l, ...
    LocationInChild, OrientationInChild);

knee_angle_l = knee_l.upd_coordinates(0);
knee_angle_l.setName('knee_angle_r');
knee_angle_l.setRange([-2.0943951, 0.17453293]);
knee_angle_l.setDefaultValue(0);
knee_angle_l.setDefaultSpeedValue(0);

tibia_l.attachGeometry(Mesh('tibia_l.vtp'));
tibia_l.attachGeometry(Mesh('l_fibula.vtp'));
myModel.addJoint(knee_l);

myModel.addBody(tibia_l);
% --------------------------------------------------------------------
%% Adding Talus_Left
talus_l = Body();
talus_l.setName('talus_l');
talus_l.setMass(0.1);
talus_l.setMassCenter(Vec3(0,0,0));
talus_l.setInertia(Inertia(0.001,0.001,0.001,0,0,0));

LocationInParent = Vec3(0,-0.43,0);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

subtalar_l = FreeJoint('ankle_joint', tibia_l, ...
    LocationInParent, OrientationInParent, ...
    talus_l, LocationInChild, OrientationInChild);

subtalar_angle_l = subtalar_l.upd_coordinates(0);
subtalar_angle_l.setName('ankle_angle_l');
subtalar_angle_l.setRange([-1.57079633, 1.57079633]);
subtalar_angle_l.setDefaultValue(0);
subtalar_angle_l.setDefaultSpeedValue(0);

talus_l.attachGeometry(Mesh('l_talus.vtp'));
myModel.addJoint(subtalar_l);

myModel.addBody(talus_l);
% --------------------------------------------------------------------
%% Adding Calcn_Left
calcn_l = Body();
calcn_l.setName('calcn_l');
calcn_l.setMass(1.25);
calcn_l.setMassCenter(Vec3(0.1,0.03,0));
calcn_l.setInertia(Inertia(0.0014,0.0039,0.0041,0,0,0));

LocationInParent = Vec3(-0.04877, -0.04195, -0.00792);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

subtalar_l = FreeJoint('subtalar_l', talus_l, ...
    LocationInParent, OrientationInParent, ...
    calcn_l, LocationInChild, OrientationInChild);

subtalar_angle_l = subtalar_l.upd_coordinates(0);
subtalar_angle_l.setName('subtalar_angle_l');
subtalar_angle_l.setRange([-1.57079633, 1.57079633]);
subtalar_angle_l.setDefaultValue(0);
subtalar_angle_l.setDefaultSpeedValue(0);

calcn_l.attachGeometry(Mesh('l_foot.vtp'));
myModel.addJoint(subtalar_l);

myModel.addBody(calcn_l);
% --------------------------------------------------------------------
%% Adding Toes_Left
toes_l = Body();
toes_l.setName('toes_l');
toes_l.setMass(0.2166);
toes_l.setMassCenter(Vec3(0.0346,0.006,0.0175));
toes_l.setInertia(Inertia(0.0001,0.0002,0.0001,0,0,0));

LocationInParent = Vec3(0.1788,-0.002,-0.00108);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

mtp_l = FreeJoint('mtp_l', calcn_l, ...
    LocationInParent, OrientationInParent, ...
    toes_l, LocationInChild, OrientationInChild);

mtp_angle_l = mtp_l.upd_coordinates(0);
mtp_angle_l.setName('mtp_angle_l');
mtp_angle_l.setRange([-1.57079633, 1.57079633]);
mtp_angle_l.setDefaultValue(0);
mtp_angle_l.setDefaultSpeedValue(0);

toes_l.attachGeometry(Mesh('l_bofoot.vtp'));
myModel.addJoint(mtp_l);

myModel.addBody(toes_l);

% ground = Body();
% ground.setName('ground');
% ground.attachGeometry(Mesh('treadmill.vtp'));


% --------------------------------------------------------------------
%% Adding Torso
torso = Body();
torso.setName('torso');
torso.setMass(34.2366);
torso.setMassCenter(Vec3(-0.03,0.32,0));
torso.setInertia(Inertia(1.47445,0.7555,1.4317,0,0,0));

LocationInParent = Vec3(-0.1007,0.0815,0);
OrientationInParent = Vec3(0,0,0);
LocationInChild = Vec3(0,0,0);
OrientationInChild = Vec3(0,0,0);

back = FreeJoint('back', pelvis, ...
    LocationInParent, OrientationInParent, torso, ...
    LocationInChild, OrientationInChild);

lumbar_extension = back.upd_coordinates(0);
lumbar_extension.setName('lumbar_extension');
lumbar_extension.setRange([-1.57079633, 1.57079633]);
lumbar_extension.setDefaultValue(0);
lumbar_extension.setDefaultSpeedValue(0);

lumbar_bending = back.upd_coordinates(1);
lumbar_bending.setName('lumbar_bending');
lumbar_bending.setRange([-1.57079633, 1.57079633]);
lumbar_bending.setDefaultValue(0);
lumbar_bending.setDefaultSpeedValue(0);

lumbar_rotation = back.upd_coordinates(2);
lumbar_rotation.setName('lumbar_rotation');
lumbar_rotation.setRange([-1.57079633, 1.57079633]);
lumbar_rotation.setDefaultValue(0);
lumbar_rotation.setDefaultSpeedValue(0);

torso.attachGeometry(Mesh('hat_spine.vtp'));
torso.attachGeometry(Mesh('hat_jaw.vtp'));
torso.attachGeometry(Mesh('hat_skull.vtp'));
torso.attachGeometry(Mesh('hat_ribs.vtp'));
myModel.addJoint(back);

myModel.addBody(torso);
% --------------------------------------------------------------------
%% Final Print of the Model
myModel.finalizeConnections();
myModel.finalizeConnections();
myModel.print('modelfixed.osim');