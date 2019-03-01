import numpy

TAU_MAX = 10.0
CURRENT_MAX = 8.
CTRL_MAX = 3.

nbJoints=1
urdfFileName="/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_quadruped/urdf/quadruped.urdf"

controlDT=0.001
maxCurrent=CURRENT_MAX
robotRef= "control-manager-robot"
urdftosot=range(nbJoints)

ctrlManagerCurrentToControlGain=1.0

ImuJointName = 'none'

mapJointNameToID={
  'Joint_1': 0,
}

mapJointLimits={
  0 : [-1.5, 1.5],
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

}

indexOfForceSensors= ()
FootFrameNames= {
  "Right": "RLEG_ANKLE_R",
  "Left" : "LLEG_ANKLE_R"
}

RightFootSensorXYZ = (0.0,0.0,0.0)
