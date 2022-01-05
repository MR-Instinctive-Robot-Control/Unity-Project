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

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    // Lock to keep the publisher from sending until successfully executed
    bool execution_lock = false;
    bool start_following = false;

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
        execution_lock = false;
        start_following = false; 
    }
    /// <summary>
    /// Called when we toggle the robot follow mode.
    /// </summary>
    public void SwitchOnOff() 
    {
        start_following = !start_following;
    }

    /// <summary>
    ///     Create a new MoveItServiceRequest to achieve the current pose of the operator's wrist.
    /// </summary>
    public void Update()
    {
        if (execution_lock || !start_following) {
            return;
        }

        var request = new MoveItPlanRequest();

        // We want to plan for a pose
        request.cmd = MoveItPlanRequest.CMD_PLAN_POSE;

        // At the moment we don't save the hand positions but query them at the button press.
        Vector3 wristP = Vector3.zero;
        Quaternion wristR = Quaternion.identity;

        // We try to get the current wrist position as the input of our PosePlanner
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            wristP = pose.Position;
            wristR = pose.Rotation;

             //geometry_msgs/Pose - requested end effector pose
            request.ee_pose = new PoseMsg
            {
                // We have to get the pose in relation with the RobotBase and transform it to the right coordinates.
                position = (pose.Position - m_RobotBase.transform.position).To<FLU>(),
                orientation = (pose.Rotation).To<FLU>()
            };

            m_Ros.Publish(m_TopicName, request.ee_pose);
            m_Ros.SendServiceMessage<MoveItPlanResponse>(m_RosServiceName, request, ExecuteTargetPose);
            // We are currently executing a trajectory and can't plan a new one
            execution_lock = true;
        }

        // We want to close the gripper when we tap our thumb and index finger and open it again if we release them.
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            wristP = pose.Position;
            wristR = pose.Rotation;

             //geometry_msgs/Pose - requested end effector pose
            request.ee_pose = new PoseMsg
            {
                // We have to get the pose in relation with the RobotBase and transform it to the right coordinates.
                position = (pose.Position - m_RobotBase.transform.position).To<FLU>(),
                orientation = (pose.Rotation).To<FLU>()
            };

            m_Ros.Publish(m_TopicName, request.ee_pose);
            m_Ros.SendServiceMessage<MoveItPlanResponse>(m_RosServiceName, request, ExecuteTargetPose);
            // We are currently executing a trajectory and can't plan a new one
            execution_lock = true;
        }

    }

    private void ExecuteTargetPose(MoveItPlanResponse response)
    {
        if (response.success) {
            var request = new MoveItPlanRequest();
            // Now we want to execute the trajectory
            request.cmd = MoveItPlanRequest.CMD_EXECUTE;
            m_Ros.SendServiceMessage<MoveItPlanResponse>(m_RosServiceName, request, UnlockRobotArm);
        } else {
            execution_lock = false;
        }
    }
    /// <summary>
    /// Lock on which we block when we are currently executing a trajectory.
    /// </summary>
    private void UnlockRobotArm(MoveItPlanResponse response)
    {
        execution_lock = false;
    }
}