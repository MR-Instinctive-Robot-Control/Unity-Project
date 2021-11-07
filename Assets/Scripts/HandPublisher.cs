using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.InterbotixMoveitInterface;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;

public class HandPublisher : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/unity/wrist_position";

    // Variables required for ROS communication
    [SerializeField]
    // service advertised by moveit_interface_obj (interbotix_moveit_interface)
    string m_RosServiceName = "/wx250s/moveit_plan";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_RobotBase;
    public GameObject RobotBase { get => m_RobotBase; set => m_RobotBase = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    // readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    // readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    //ArticulationBody m_LeftGripper;
    //ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    MixedRealityPose pose;
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoveItPlanRequest, MoveItPlanResponse>(m_RosServiceName);
        m_Ros.RegisterPublisher<PoseMsg>(m_TopicName);
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PlanAndExecuteTargetPose()
    {
        var request = new MoveItPlanRequest();

        // plan pose = 
        request.cmd = MoveItPlanRequest.CMD_PLAN_POSE;

        // At the moment we don't save the hand positions but query them at the button press.
        Vector3 wristP = Vector3.zero;
        Quaternion wristR = Quaternion.identity;

        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            wristP = pose.Position;
            wristR = pose.Rotation;

             //geometry_msgs/Pose - requested end effector pose
            request.ee_pose = new PoseMsg
            {
                position = (pose.Position - m_RobotBase.transform.position).To<FLU>(),

                orientation = (pose.Rotation).To<FLU>()
                // orientation =  Quaternion.Euler(0, 0, 0).To<FLU>()
            };

            m_Ros.Publish(m_TopicName, request.ee_pose);
            m_Ros.SendServiceMessage<MoveItPlanResponse>(m_RosServiceName, request, ExecuteTargetPose);
        }
    }

    public void ExecuteTargetPose(MoveItPlanResponse response)
    {
        if (response.success) {
            var request = new MoveItPlanRequest();
            request.cmd = MoveItPlanRequest.CMD_EXECUTE;
            m_Ros.SendServiceMessage<MoveItPlanResponse>(m_RosServiceName, request);
        }
    }
}