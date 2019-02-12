import numpy

TAU_MAX = 10.0
CURRENT_MAX = 3.
CTRL_MAX = 3.

nbJoints=8
urdfFileName="/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_quadruped/urdf/quadruped.urdf"
controlDT=0.001
maxCurrent=CURRENT_MAX
robotRef= "control-manager-robot"
urdftosot=range(nbJoints)

ctrlManagerCurrentToControlGain=1.0

ImuJointName = 'none'

mapJointNameToID={
  'FL_HFE': 0,
  'FL_KFE': 1,
  'FR_HFE': 2,
  'FR_KFE': 3,
  'HL_HFE': 4,
  'HL_KFE': 5,
  'HR_HFE': 6,
  'HR_KFE': 7,
}

mapJointLimits={
  0 : [-1.5, 1.5],
  1 : [-1.5, 1.5],
  2 : [-1.5, 1.5],
  3 : [-1.5, 1.5],
  4 : [-1.5, 1.5],
  5 : [-1.5, 1.5],
  6 : [-1.5, 1.5],
  7 : [-1.5, 1.5],
}

fMax=numpy.array([100.0,100.0,300.0,80.0,80.0,30.0])
fMin=-fMax
mapForceIdToForceLimits={
  0: [fMin,fMax],
  1: [fMin,fMax],
  2: [fMin,fMax],
  3: [fMin,fMax]
}

mapNameToForceId={
  "rf": 0,
  "lf": 1,
  "rh": 2,
  "lh": 3
}

indexOfForceSensors= ()
FootFrameNames= {
  "Right": "RLEG_ANKLE_R",
  "Left" : "LLEG_ANKLE_R"
}

RightFootSensorXYZ = (0.0,0.0,0.0)
