using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class PositionSubscriber : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 9;

    //public static readonly string[] LinkNames =
    //    { "upper_arm_link" }; //, "/shoulder_link", "/upper_arm_link", "/upper_forearm_link", "lower_forearm_link", "/wrist_link"

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    GameObject[] m_Links;

    [SerializeField]
    GameObject m_Shoulder; //waist

    [SerializeField]
    GameObject m_UpperArm; //shoulder

    [SerializeField]
    GameObject m_UpperForearm; //elbow

    [SerializeField]
    GameObject m_LowerForearm; //forearm_roll

    [SerializeField]
    GameObject m_WristLink; //wrist_angle
    
    [SerializeField]
    GameObject m_GripperLink; // wrist_rotate

    [SerializeField]
    GameObject m_GripperProp; //gripper

    [SerializeField]
    GameObject m_RightFinger;

    [SerializeField]
    GameObject m_LeftFinger;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        m_Links = new GameObject[]{m_UpperForearm, m_LowerForearm, m_GripperProp,
                m_LeftFinger, m_RightFinger, m_UpperArm, m_Shoulder, m_WristLink,
                m_GripperLink};
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        for (var i = 0; i < m_Links.Length; i++)
        {
            m_JointArticulationBodies[i] = m_Links[i].GetComponent<ArticulationBody>();
        }

        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.Subscribe<JointStateMsg>("/wx250s/joint_states", PositionCallback);
        Debug.Log("Subscriber initialized...");
    }

    void PositionCallback(JointStateMsg msg)
    {   
        // Set the joint values for every joint
        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
        {
            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = (float)msg.position[joint] * Mathf.Rad2Deg;
            m_JointArticulationBodies[joint].xDrive = joint1XDrive;   
        }
    }
}
