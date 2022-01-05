using System;
using System.Collections.Generic;
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
    //ArticulationBody[] m_JointArticulationBodies;
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

    IDictionary<string, ArticulationBody> m_JointArticulationBodies;

    // float[] previous_error = new float[] { 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void Start()
    {
        m_JointArticulationBodies = new Dictionary<string, ArticulationBody>();
        m_JointArticulationBodies.Add("waist", m_Shoulder.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("shoulder", m_UpperArm.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("elbow", m_UpperForearm.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("forearm_roll", m_LowerForearm.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("wrist_angle", m_WristLink.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("wrist_rotate", m_GripperLink.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("gripper", m_GripperProp.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("left_finger", m_LeftFinger.GetComponent<ArticulationBody>());
        m_JointArticulationBodies.Add("right_finger", m_RightFinger.GetComponent<ArticulationBody>());

        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.Subscribe<JointStateMsg>("/wx250s/joint_states", PositionCallback);
        Debug.Log("Subscriber initialized...");
    }

    void PositionCallback(JointStateMsg msg)
    {   
        // Set the joint values for every joint
        // var virtual_ArticulationBodies = m_JointArticulationBodies;
        // bool execute = true;
        for (var joint = 0; joint < msg.name.Length; joint++)
        {
            string jointName = msg.name[joint];
            var joint1XDrive = m_JointArticulationBodies[jointName].xDrive;
            if (jointName.Contains("finger")) {
                joint1XDrive.target = (float)msg.position[joint];
                m_JointArticulationBodies[jointName].xDrive = joint1XDrive;
            } else {
                 joint1XDrive.target = (float)msg.position[joint] * Mathf.Rad2Deg;
                m_JointArticulationBodies[jointName].xDrive = joint1XDrive;
            }
           

            /*______ TRIED TO FILTER BUT IT SUCKED : ____*/
            // string jointName = msg.name[joint];
            // var joint1XDrive = virtual_ArticulationBodies[jointName].xDrive;
            // float previous_target = joint1XDrive.target;
            // float this_target =  (float)msg.position[joint] * Mathf.Rad2Deg;
            // float error = this_target - previous_target;
            // // limit the error to vary by maximum 0.1 between updates
            // if (Math.Abs(error - previous_error[joint]) < 0.14f) {
            //     joint1XDrive.target = this_target;
            //     virtual_ArticulationBodies[jointName].xDrive = joint1XDrive;
            //     previous_error[joint] = error; 
            // } else {
            //     previous_error[joint] += 0.1f * (error-previous_error[joint]);
            //     execute = false;
            // }
            
        }
        // if (execute) {
        //     m_JointArticulationBodies = virtual_ArticulationBodies;
        // }
    }
}
