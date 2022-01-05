using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
// using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class GenericPublisher : MonoBehaviour
{
    public static readonly string LinkName = "wx250s/wrist_link";

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/unity/wrist_position";

    [SerializeField]
    GameObject m_Robot;

    // Wrist Joint
    // UrdfJointRevolute m_JointArticulationBody;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<PoseMsg>(m_TopicName);

        // m_JointArticulationBody = m_Robot.transform.Find(LinkName).GetComponent<Transform>();
    }


    // Update gets called on everyframe, whereas a generic functions needs to be 
    // called through some other action, like for example a button click
    public void Publish()
    {
        // create the message
        var msg = new PoseMsg
        {
            position =  m_Robot.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Robot.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, msg);
    }
}